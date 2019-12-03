package net.imglib2.realtransform;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import net.imglib2.Cursor;
import net.imglib2.FinalInterval;
import net.imglib2.Interval;
import net.imglib2.RandomAccess;
import net.imglib2.RandomAccessibleInterval;
import net.imglib2.RealLocalizable;
import net.imglib2.RealPositionable;
import net.imglib2.iterator.IntervalIterator;
import net.imglib2.type.numeric.RealType;
import net.imglib2.util.Intervals;
import net.imglib2.util.Util;
import net.imglib2.view.IntervalView;
import net.imglib2.view.Views;

// TODO document better
public class BSplineRealTransform< T extends RealType<T> > implements RealTransform
{
	final int numDimensions;

	private final int order;

	private final BSplineKernel kernel;

	// grid
	private final double[] gridOffset;
	private final double[] gridSpacing;

	final List<RandomAccessibleInterval<T>> coefficientImages;

	final RandomAccessibleInterval<T> weightImage;

	public BSplineRealTransform( 
			final int numDimensions, 
			final int order, 
			final List<RandomAccessibleInterval<T>> coefficients,
			final double[] gridSpacing,
			final double[] gridOffset )
	{
		this.order = order;
		this.gridSpacing = gridSpacing;
		this.gridOffset = gridOffset;

		kernel = new BSplineKernel( order );
		this.numDimensions = coefficients.get( 0 ).numDimensions();

		this.coefficientImages = coefficients;

		long[] supportSize = new long[ numDimensions ];
		Arrays.fill( supportSize, (long)kernel.supportWidth() );

		T type = Views.iterable( coefficients.get( 0 )).firstElement().copy();
		weightImage = Util.getSuitableImgFactory( new FinalInterval( supportSize ), type ) .create( supportSize );
	}

	public RandomAccessibleInterval< T > getWeights()
	{
		return weightImage;
	}

	public void buildWeights( double[] relativePosition )
	{
		// TODO I'm sure this could be optimized
		// (but it should only be called once), so don't bother unless i
		// it's shown to be a problem

		Cursor<T> c = Views.iterable( weightImage ).cursor();
		for( int d = 0; d < numDimensions; d++ )
		{
			c.reset();
			while( c.hasNext() )
			{
				c.fwd();
				if( d == 0 )
				{
					c.get().setOne();
				}
				double w = kernel.evaluate( relativePosition[ d ] - c.getDoublePosition( d ) );
				c.get().mul( w );
			}
		}
	}

	@Override
	public int numSourceDimensions()
	{
		return numDimensions;
	}

	@Override
	public int numTargetDimensions()
	{
		return numDimensions;
	}

	/*
	 * see
	 * Common/Transforms/itkAdvancedBSplineDeformableTransform.hxx
	 * line 222 
	 * 
	 */
	@Override
	public void apply( double[] source, double[] target )
	{
		// temporary variables - move these to class members if parallelism doesn't matter
		double[] displacement = new double[ numDimensions ];

		long[] startIndex = new long[ numDimensions  ];
		long[] endIndex = new long[ numDimensions  ];
		double[] ptPixelSpace = new double[ numDimensions ];
		double[] ptPixRelativeToSupport = new double[ numDimensions ];

		int d = 0;
		for( d = 0; d < numDimensions; d++ )
		{
			ptPixelSpace[ d ] = ( source[ d ] - gridOffset[ d ] ) / gridSpacing[ d ];

			startIndex[ d  ] = ( int ) Math.ceil( ptPixelSpace[ d ] - kernel.supportRadius() );
			endIndex[ d ] = startIndex[ d ] + kernel.supportWidth() - 1;

			ptPixRelativeToSupport[ d ] = ptPixelSpace[ d ] - startIndex[ d ];
		}
		final Interval kernelSupport = new FinalInterval( startIndex, endIndex );

		buildWeights( ptPixRelativeToSupport );

		// TODO
		// zero displacement if kernel support is not in valid region
		if( !isKernelInValidRegion(kernelSupport))
		{
			System.arraycopy(source, 0, target, 0, target.length);
			return;
		}

		List< IntervalView< T > > coefsInSupport = coefficientImages.stream().map( x -> Views.interval( Views.extendZero( x ), kernelSupport ) ).collect( Collectors.toList() );
		List< RandomAccess< T > > coefsInSupportRas = coefficientImages.stream().map( x -> Views.interval( Views.extendZero( x ), kernelSupport ).randomAccess() ).collect( Collectors.toList() );

		//IntervalView< T > coefficientSupport = Views.interval( coefficientImage, kernelSupport );
		IntervalIterator spatialIt = new IntervalIterator( coefsInSupport.get( 0 ) );
		RandomAccess< T > wRa = Views.translate( weightImage, Intervals.minAsLongArray( kernelSupport )).randomAccess();

		while( spatialIt.hasNext() )
		{
			spatialIt.fwd();
			wRa.setPosition( spatialIt );

			for( d = 0; d < numDimensions; d++ )
			{
				double w = wRa.get().getRealDouble();

				// TODO could be more efficient here
				coefsInSupportRas.get( d ).setPosition( spatialIt );
				double coef = coefsInSupportRas.get( d ).get().getRealDouble();

				displacement[ d ] += w * coef;
			}
		}

		// TODO copy source to target first and incrementally update above
		// then we dont need displacement var

		// displace source and set result to target
		for( d = 0; d < numDimensions; d++ )
			target[ d ] = source[ d ] + displacement[ d ];
	}

	@Override
	public void apply( RealLocalizable source, RealPositionable target )
	{
		double[] s = new double[ source.numDimensions() ];
		double[] t = new double[ target.numDimensions() ];
		source.localize( s );
		apply( s, t );
		target.setPosition( t );
	}

	@Override
	public RealTransform copy()
	{
		return new BSplineRealTransform<T>(this.numDimensions, this.order, this.coefficientImages,
				this.gridSpacing, this.gridOffset);
	}

	/**
	 * Returns the interval with one fewer dimension resulting after removing 
	 * a specified dimension
	 * 
	 * @param interval the interval
	 * @param dim the dimension to remove
	 * @return the lower dimensional interval
	 */
	public static Interval intervalHyperSlice( Interval interval, int dim )
	{
		int nd = interval.numDimensions() - 1;
		long[] min = new long[ nd ];
		long[] max = new long[ nd ];

		int dout = 0;
		for( int dimIn = 0; dimIn < interval.numDimensions(); dimIn++ )
		{
			if( dimIn == dim )
			{
				dimIn++;
				continue;
			}
			else
			{
				min[ dout ] = interval.min( dimIn );
				max[ dout ] = interval.max( dimIn );
				dimIn++;
				dout++;
			}
		}
		return new FinalInterval( min, max );
	}

	public boolean isKernelInValidRegion( final Interval kernelSupport )
	{
		for( int d = 0; d < kernelSupport.numDimensions(); d++ )
		{
			if( kernelSupport.min( d ) < 0 || 
				kernelSupport.max(d) > coefficientImages.get(0).max(d) )
			{
				return false;
			}
		}
		return true;
	}

	public Interval computeKernelSupport( final double[] pixelCoordinate )
	{
		long[] startIndex = new long[ numDimensions  ];
		long[] endIndex = new long[ numDimensions  ];

		for( int d = 0; d < numDimensions; d++ )
		{
			startIndex[ d  ] = ( int ) Math.ceil( pixelCoordinate[ d ] - kernel.supportRadius() );
			endIndex[ d ] = startIndex[ d ] + kernel.supportWidth() - 1;
		}
		return new FinalInterval( startIndex, endIndex );
	}

	public void transformPointToContinuousGridIndex( final double[] p, final int[] gridIndex )
	{
		double[] tvector = new double[ numDimensions ];
		for( int d = 0; d < numDimensions; d++ )
		{
			tvector[ d ] = p[ d ] - gridOffset[ d ];
		}

		for( int d = 0; d < numDimensions; d++ )
		{
			gridIndex[ d ] = (int)Math.round( tvector[ d ] );
		}
	}

/**
 * 
 * Port of itkBSplineKernelFunction2
 *
 */
public static class BSplineKernel
{
	private final int order;

	public BSplineKernel( final int order )
	{
		this.order = order;
	}

	public double evaluate( final double u )
	{
		switch ( order )
		{
		case 0:
			return evaluate0( u );
		case 1:
			return evaluate1( u );
		case 2:
			return evaluate2( u );
		case 3:
			return evaluate3( u );
		}
		return Double.NaN;
	}

	public double supportRadius()
	{
		return ( (double)order + 1.0 ) / 2.0;
	}

	public int supportWidth()
	{
		return order + 1;
	}

	/*
	 * Zeroth order spline kernel
	 */
	public static double evaluate0( final double u )
	{
		final double absValue = Math.abs( u );
		if ( absValue < 0.5 )
			return 1.0;
		else if ( absValue == 0.5 )
			return 0.5;
		else
			return 0.0;
	}

	/*
	 * First order spline kernel
	 */
	public static double evaluate1( final double u )
	{
		final double absValue = Math.abs( u );
		if ( absValue < 0.5 )
			return 1.0 - absValue;
		else
			return 0.0;
	}

	/*
	 * Second order spline kernel
	 */
	public static double evaluate2( final double u )
	{
		final double absValue = Math.abs( u );
		if ( absValue < 1.0 )
			return 0.75 - absValue * absValue;
		else if ( absValue < 1.5 )
			return ( 9.0 - 12.0 * absValue + 4.0 * absValue * absValue ) / 8.0;
		else
			return 0.0;
	}

	/*
	 * Third order spline kernel
	 */
	public static double evaluate3( final double u )
	{
		final double absValue = Math.abs( u );
		final double sqrValue = u * u;
		if ( absValue < 1.0 )
			return ( 4.0 - 6.0 * sqrValue + 3.0 * sqrValue * absValue ) / 6.0;
		else if ( absValue < 2.0 )
			return ( 8.0 - 12.0 * absValue + 6.0 * sqrValue - sqrValue * absValue ) / 6.0;
		else
			return 0.0;
	}
}

}
