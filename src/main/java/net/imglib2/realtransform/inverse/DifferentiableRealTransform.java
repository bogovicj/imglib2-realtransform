package net.imglib2.realtransform.inverse;

import net.imglib2.realtransform.RealTransform;

/**
 * Transformation that can explicitly compute its derivative (jacobian).
 * <p>
 * {@link DifferentiableRealTransform}s can have their inverse iteratively
 * estimated by gradient descent, see {@link BacktrackingLineSearch}. 
 * 
 * 
 * @author John Bogovic &lt;bogovicj@janelia.hhmi.org&gt;
 *
 */
public interface DifferentiableRealTransform extends RealTransform
{
	/**
	 * Writes the direction <em>displacement</em> in which to move the input source 
	 * point <em>x</em> in order that F( x + d ) is closer to the destination
	 * point <em>y</em> than F( x ).
	 * <p>
	 * The output is a normalized vector.
	 * 
	 * @param displacement the displacement to write into
	 * @param x the source point
	 * @param y the destination point
 	 * @return the direction
	 */
	public void directionToward( final double[] displacement, final double[] x, final double[] y );
}