package net.imglib2.realtransform.inverse;

import net.imglib2.realtransform.AffineTransform;

/**
 * Never use this. This thing's main use is to help test BacktrackingLineSearch
 * 
 * @author John Bogovic
 *
 */
public class IterativeAffineInverse extends AffineTransform implements DifferentiableRealTransform
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