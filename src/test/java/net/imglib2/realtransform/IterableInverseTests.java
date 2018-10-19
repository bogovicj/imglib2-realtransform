package net.imglib2.realtransform;

import java.util.Arrays;

import org.junit.Assert;
import org.junit.Test;

import net.imglib2.realtransform.inverse.DifferentiableRealTransform;
import net.imglib2.realtransform.inverse.InverseRealTransformGradientDescent;
import net.imglib2.realtransform.inverse.RealTransformFiniteDerivatives;
import net.imglib2.realtransform.inverse.WrappedIterativeInvertibleRealTransform;

public class IterableInverseTests
{
	public final double EPS = 0.0001;

	@Test
	public void testTpsInverse()
	{
		double[][] srcPts = new double[][]{
				{ -1.0, 0.0, 1.0, 0.0 }, // x
				{ 0.0, -1.0, 0.0, 1.0 } }; // y

		double[][] tgtPts = new double[][]{
				{ -2.0, 0.0, 2.0, 0.0 }, // x
				{ 0.0, -2.0, 0.0, 2.0 } }; // y


		ThinplateSplineTransform tps = new ThinplateSplineTransform( srcPts, tgtPts );
		WrappedIterativeInvertibleRealTransform<ThinplateSplineTransform> tpsInv = new WrappedIterativeInvertibleRealTransform<>( tps );
		
		
		double[] x = new double[]{ 0.5, 0.5 };
		double[] y  = new double[ 2 ];
		double[] yest = new double[ 2 ];
		double[] yestinv = new double[ 2 ];
		
		tpsInv.getOptimzer().setTolerance( EPS / 2 );
		
		tps.apply( x, y );
		tpsInv.applyInverse( yest, y );
		tps.apply( yest, yestinv );
		
//		System.out.println( "x      : " + Arrays.toString( x ));
//		System.out.println( "y      : " + Arrays.toString( y ));
//		System.out.println( "yest   : " + Arrays.toString( yest ));
//		System.out.println( "yesti  : " + Arrays.toString( yestinv ));

		Assert.assertTrue( true );
		Assert.assertArrayEquals("tps", x, yest, EPS );
	}

	@Test
	public void testAffineInverse()
	{
		double[] p = new double[]{ 3, 0.5, 30 };
		double[] pxfm = new double[ 3 ];
		double[] q = new double[ 3 ];

		IterativeAffineInverse I3 = new IterativeAffineInverse( 3 );
		
		
		I3.apply( p, pxfm );
		I3.applyInverse( q, p );
		Assert.assertArrayEquals( "identity matrix inverse", pxfm, q, EPS );

		System.out.println( "q : " + Arrays.toString( q ));
		
		IterativeAffineInverse rot = new IterativeAffineInverse( 3 );

		rot.set( 0.0, 1.0, 0.0, 0.1,
				 -1.0, 0.0, 0.0, 8,
				 0.0, 0.0, 1.0, 50 );
		
		
		// a difficult case in which
		// the optimizer must 
		InverseRealTransformGradientDescent rotinverter = new InverseRealTransformGradientDescent( 
				3, rot );
		rotinverter.setTolerance( EPS / 20 );

		rot.apply( p, pxfm );
		rotinverter.apply( pxfm, q );


		Assert.assertArrayEquals( "rotation matrix inverse", p, q, EPS );
	}
	
	
	private class IterativeAffineInverse extends AffineTransform implements DifferentiableRealTransform
	{
		final AffineTransform jacobian;

		// TODO move this class into tests
		public IterativeAffineInverse( int n )
		{
			super( n );
			jacobian = new AffineTransform( n );
		}

		@Override
		public void directionToward( double[] displacement, double[] x, double[] y )
		{
			RealTransformFiniteDerivatives.directionToward( jacobian(x), displacement, x, y );
		}

		@Override
		public AffineTransform jacobian( double[] x )
		{
			jacobian.set( this.getRowPackedCopy() );
			for( int d = 0; d < n; d++ )
				jacobian.set( 0.0, d, n );

			//System.out.println( "jac      : " + Arrays.toString( jacobian.getRowPackedCopy() ));

			return jacobian;
		}

	}
}
