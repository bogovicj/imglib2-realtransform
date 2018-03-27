package net.imglib2.realtransform.inverse;

import net.imglib2.realtransform.AffineTransform;

/**
 * Never use this. 
 * This thing's main use is to help test BacktrackingLineSearch
 * 
 * @author bogovicj
 *
 */
public class IterativeAffineInverse extends AffineTransform implements DifferentiableRealTransform
{

	public IterativeAffineInverse( int n )
	{
		super( n );
		
	}

	@Override
	public void directionToward( double[] displacement, double[] x, double[] y )
	{
		// use displacement temporarily to store Ax
		this.apply( x, displacement );
		
		// now displacement holds (Ax - y )
		for( int i = 0; i < displacement.length; i++ )
		{
			displacement[ i ] -= y[ i ];
		}
	}

	@Override
	public AffineTransform jacobian(double[] x) {
		// TODO Auto-generated method stub
		return null;
	}

}
