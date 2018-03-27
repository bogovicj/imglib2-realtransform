package net.imglib2.realtransform.inverse;

import net.imglib2.realtransform.AffineTransform;
import net.imglib2.realtransform.RealTransform;

/**
 * Transformation that can explicitly compute its derivative (jacobian).
 * <p>
 * {@link DifferentiableRealTransform}s can have their inverse iteratively
 * estimated by gradient descent, see
 * {@link InverseRealTransformGradientDescent}.
 * 
 * 
 * @author John Bogovic &lt;bogovicj@janelia.hhmi.org&gt;
 *
 */
public abstract class DifferentiableRealTransform implements RealTransform
{

	/**
	 * Returns the jacobian matrix of this transform at the point x.
	 * 
	 * @param x
	 *            the point
	 * @return the jacobian
	 */
	public abstract AffineTransform jacobian( double[] x );

	/**
	 * Writes the direction <em>displacement</em> in which to move the input
	 * source point <em>x</em> in order that F( x + d ) is closer to the
	 * destination point <em>y</em> than F( x ).
	 * <p>
	 * The output is a normalized vector.
	 * 
	 * @param displacement
	 *            the displacement to write into
	 * @param x
	 *            the source point
	 * @param y
	 *            the destination point
	 * @return the direction
	 */
	public void directionToward( final double[] displacement, final double[] x, final double[] y )
	{
		double[] err = new double[ x.length ];
		for ( int i = 0; i < x.length; i++ )
			err[ i ] = y[ i ] - x[ i ];

		AffineTransform j = jacobian( x );
		double[] dir = new double[ x.length ];
		j.inverse().apply( err, dir );

		double norm = 0.0;
		for ( int i = 0; i < dir.length; i++ )
			norm += ( dir[ i ] * dir[ i ] );

		norm = Math.sqrt( norm );

		for ( int i = 0; i < dir.length; i++ )
			dir[ i ] /= norm;

		// compute the directional derivative
		double[] directionalDerivative = new double[ dir.length ];
		j.apply( dir, directionalDerivative );

		double descentDirectionMag = 0.0;
		for ( int i = 0; i < dir.length; i++ )
			descentDirectionMag += ( dir[ i ] * directionalDerivative[ i ] );
	}

}
