/*
 * #%L
 * ImgLib2: a general-purpose, multidimensional image processing library.
 * %%
 * Copyright (C) 2009 - 2016 Tobias Pietzsch, Stephan Preibisch, Stephan Saalfeld,
 * John Bogovic, Albert Cardona, Barry DeZonia, Christian Dietz, Jan Funke,
 * Aivar Grislis, Jonathan Hale, Grant Harris, Stefan Helfrich, Mark Hiner,
 * Martin Horn, Steffen Jaensch, Lee Kamentsky, Larry Lindsey, Melissa Linkert,
 * Mark Longair, Brian Northan, Nick Perry, Curtis Rueden, Johannes Schindelin,
 * Jean-Yves Tinevez and Michael Zinsmaier.
 * %%
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * #L%
 */

package net.imglib2.realtransform;

import java.util.Arrays;

import net.imglib2.RandomAccessibleInterval;
import net.imglib2.RealLocalizable;
import net.imglib2.RealPositionable;
import net.imglib2.RealRandomAccess;
import net.imglib2.RealRandomAccessible;
import net.imglib2.interpolation.randomaccess.NLinearInterpolatorFactory;
import net.imglib2.type.numeric.RealType;
import net.imglib2.view.Views;

/**
 * An <em>n</em>-dimensional deformation field.
 * <p> 
 * Wraps a {@link RandomAccessibleInterval} of dimensionality D+1 ("def")
 * and interprets it as a D-dimensional {@link RealTransform}.  The last
 * dimension of of the {@link RandomAccessibleInterval} must have at
 * least D components.
 * <p>
 * The deformation field should be interpreted as a d-dimensional
 * vector field.  A source point is displaced by adding the vector
 * at that point the the source point's position. 
 * 
 * @author John Bogovic &lt;bogovicj@janelia.hhmi.org&gt;
 *
 */
public class InvertibleDeformationFieldTransform<T extends RealType<T>> implements InvertibleRealTransform
{
	int lineSearchMaxTries = 16;
	double c = 0.5;
	double beta = 0.5;

	int maxIters = 200;
	double tolerance = 0.25;

	private final int numDim;

	private final DeformationFieldTransform< T > def;

	public InvertibleDeformationFieldTransform( DeformationFieldTransform< T > def )
	{
		this.def = def;
		numDim = def.numSourceDimensions();
	}
	
	public InvertibleDeformationFieldTransform( RandomAccessibleInterval< T > def )
	{
		this( new DeformationFieldTransform<>( def ));
	}

	public InvertibleDeformationFieldTransform( RealRandomAccessible< T > defFieldReal )
	{
		this( new DeformationFieldTransform<>( defFieldReal ));
	}
	
	public void setTolerance( final double tol )
	{
		tolerance = tol;
	}
	
	public void setMaxIters( final int maxIters )
	{
		this.maxIters = maxIters;
	}

	@Override
	public int numSourceDimensions()
	{
		return def.numSourceDimensions();
	}

	@Override
	public int numTargetDimensions()
	{
		return def.numTargetDimensions();
	}

	@Override
	public void apply( double[] source, double[] target )
	{
		def.apply( source, target );
	}

	@Override
	public void apply( float[] source, float[] target )
	{
		def.apply( source, target );
	}

	@Override
	public void apply( RealLocalizable source, RealPositionable target )
	{
		def.apply( source, target );
	}
	
	public void applyInverse( double[] source, double[] target )
	{
		applyInverse( source, target, target, 1.0 );
	}
	
	public void applyInverse( double[] source, double[] target, double[] guess, double initStepSize )
	{
		
		// start at the target position
		System.arraycopy( guess, 0, source, 0, guess.length );
		double stepSize = initStepSize;
		
		//double[] estimate = new double[ this.numSourceDimensions() ];
		double[] displacement = new double[ this.numSourceDimensions() ];
		double[] srcXfm = new double[ this.numSourceDimensions() ];
		double[] tmp = new double[ this.numSourceDimensions() ];
		
//		double squareTolerance = tolerance * tolerance;
		RealRandomAccess< T > defAccess = def.getDefFieldAcess();
		
		DefFieldInverter inv = new DefFieldInverter( source.length );
		inv.setTarget( target );

		int i = 0;
		while( i < maxIters )
		{
			inv.setEstimate( source );
			
			apply( source, tmp );
			double squaredError = inv.squaredErrorAt( source );
			double olderror = Math.sqrt( squaredError );
			System.out.println( "    old guess: " + Arrays.toString( source ));
			System.out.println( "      goes to: " + Arrays.toString( tmp ));
			
			// return right away if the initial estimate is good enough
			if( olderror < tolerance )
			{
				System.out.println( "returning with error: " + olderror );
				return;
			}
			
			// get the displacement vector here
			defAccess.setPosition( source[ 0 ], 0 );
			defAccess.setPosition( source[ 1 ], 1 );
			defAccess.setPosition( source[ 2 ], 2 );
			defAccess.setPosition( 0.0, numDim );
			for ( int d = 0; d < numDim; d++ )
			{
				displacement[ d ] = -defAccess.get().getRealDouble();
				defAccess.fwd( numDim );
			}
			inv.setDirection( displacement );
			System.out.println( "old step size: " + stepSize );
			stepSize = inv.backtrackingLineSearch( c, beta, lineSearchMaxTries, stepSize );
			System.out.println( "new step size: " + stepSize );


			for ( int d = 0; d < numDim; d++ )
			{
				tmp[ d ] = source[ d ] + stepSize * displacement[ d ]; 
			}

			apply( tmp, srcXfm );
			squaredError = inv.squaredError( srcXfm );
			double error = Math.sqrt( squaredError );
			
			if( error > olderror )
			{
				System.out.println( "could not improve...returning" );
				return;
			}
			else
			{
				System.arraycopy( tmp, 0, source, 0, source.length );
			}
			
			System.out.println( "    new guess: " + Arrays.toString( source ));
			System.out.println( "      goes to: " + Arrays.toString( srcXfm ));
			System.out.println( "       target: " + Arrays.toString( target ));
			System.out.println( "old error: " + olderror );
			System.out.println( "new error: " + error );
			System.out.println( " " );
			
			if( error < tolerance )
				break;
			
			i++;
		}
	}
	
	protected class DefFieldInverter
	{
		double[] target;
		double[] x;
		double[] y;
		
		double fx;
		double[] x_ap;
		double[] y_ap;
		double[] dir;
		int nd;
		
		double currentSquaredError;
		
		
		public DefFieldInverter( int nd )
		{
			this.nd = nd;
			y = new double[ nd ];
			x_ap = new double[ nd ];
			y_ap = new double[ nd ];
		}

		public void setEstimate( double[] est )
		{
			this.x = est;
			apply( x, y );
			fx = squaredError( y );
		}
		public void setTarget( double[] tgt )
		{
			this.target = tgt;
		}
		public void setDirection( double[] dir )
		{
			this.dir = dir;
			System.out.println( "    dir     : " + Arrays.toString( dir ));
			double mag = dirMag();
			for ( int i = 0; i < nd; i++ )
				dir[ i ] = dir[ i ] / mag;
			System.out.println( "    dir norm: " + Arrays.toString( dir ));
		}
		
		private double squaredError( double[] estimate )
		{
			double squaredError = 0;
			for ( int d = 0; d < nd; d++ )
			{
				squaredError += ( estimate[ d ] - target[ d ]) * ( estimate[ d ] - target[ d ]);
			}
			return squaredError;
		}
		
		private double squaredErrorAt( double[] estimate )
		{
			double[] srcXfm = new double[ nd ];
			apply( estimate, srcXfm );
			double squaredError = 0;
			for ( int d = 0; d < nd; d++ )
			{
				squaredError += ( srcXfm[ d ] - target[ d ]) * ( srcXfm[ d ] - target[ d ]);
			}
			return squaredError;
		}
		
		public double dirMag()
		{
			double mag = 0;
			for ( int i = 0; i < nd; i++ )
				mag += dir[ i ] * dir[ i ];
			
			return Math.sqrt( mag );
		}
		
		/**
		 * Take from jitk tps
		 * 
		 * @param c the armijoCondition parameter
		 * @param beta the fraction to multiply the step size at each iteration ( < 1 )
		 * @param maxtries max number of tries
		 * @param t0 initial step size
		 * @return the step size
		 */
		public double backtrackingLineSearch( double c, double beta, int maxtries, double t0 )
		{
			double t = t0; // step size
	
			int k = 0;
			// boolean success = false;
			while ( k < maxtries )
			{
				if ( armijoCondition( c, t ) )
				{
					// success = true;
					break;
				}
				else
					t *= beta;
	
				k++;
			}
	
			System.out.println( "selected step size after " + k + " tries" );
	
			return t;
		}
		
		private double sumSquaredErrorsDeriv( double[] y, double[] x )
		{
			double errDeriv = 0.0;
			for ( int i = 0; i < nd; i++ )
				errDeriv += ( y[ i ] - x[ i ] ) * ( y[ i ] - x[ i ] );

			return 2 * errDeriv;
		}
		
		private boolean armijoCondition( double c, double t )
		{
			for ( int i = 0; i < x.length; i++ )
				x_ap[ i ] = x[ i ] + t * dir[ i ];
			
			apply( x_ap, y_ap );
			double fx_ap = squaredError( y_ap );
			
			//double m = sumSquaredErrorsDeriv( this.target, y_ap ); // * descentDirectionMag.get( 0 );
			double m = 1;

//			System.out.println( "   x   : " + Arrays.toString( x ));
//			System.out.println( "   x_ap: " + Arrays.toString( x_ap ));
//			System.out.println( "   y   : " + Arrays.toString( y ));
//			System.out.println( "   y_ap: " + Arrays.toString( y_ap ));
//			System.out.println( "   fx      : " + fx );
//			System.out.println( "   fx_ap   : " + fx_ap );
//			System.out.println( "   fx + ctm: " + ( fx - (c*t*m)) ) ;

			if ( fx_ap < fx - c * t * m )
			{
				currentSquaredError = fx_ap;
				return true;
			}
			else
				return false;
		}
	}

	@Override
	public void applyInverse( float[] source, float[] target )
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public void applyInverse( RealPositionable source, RealLocalizable target )
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public InvertibleRealTransform inverse()
	{
		return null;
	}

	@Override
	public InvertibleRealTransform copy()
	{
		return this;
	}


}
