package net.imglib2.realtransform.interpolation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import net.imglib2.RealLocalizable;
import net.imglib2.RealPoint;
import net.imglib2.RealPositionable;
import net.imglib2.realtransform.RealTransform;
import net.imglib2.type.numeric.real.AbstractRealType;

public class LinearCombinationRealTransform extends AbstractRealType<LinearCombinationRealTransform> implements RealTransform {
	
//	private final Map<RealTransform,DoubleType> transforms;

//	private List<WeightedRealTransfom> transforms;

	private List<RealTransform> transforms;

	private List<Double> weights;
	
	private double[] tmp;

	private RealPoint tmpPt;

	private RealPoint accumulator;

	private int nd = 0; 
	
	private double w = 1.0;

	public LinearCombinationRealTransform( int nd ) { 
		this.nd = nd;
		tmp = new double[ nd ];
		tmpPt = RealPoint.wrap(tmp);
		accumulator = new RealPoint( tmp.length );
		transforms = new ArrayList<>();
		weights = new ArrayList<>();
	}

//	public LinearCombinationRealTransfom(RealTransform t) {
//		this(t.numSourceDimensions());
//		if (t != null) {
//			if (t instanceof WeightedRealTransfom)
//				transforms.add((WeightedRealTransfom) t);
//			else
//				transforms.add(new WeightedRealTransfom(t));
//		}
//	}
//	
//	private LinearCombinationRealTransfom( List<WeightedRealTransfom> ts ) { 
//		transforms = new ArrayList<>();
//		transforms.addAll(ts);
//	}

	public LinearCombinationRealTransform(RealTransform t) {
		this(t.numSourceDimensions());
		if (t != null) {
//			if (t instanceof LinearCombinationRealTransform)
//				transforms.add((LinearCombinationRealTransform) t);
//			else
//				transforms.add(new LinearCombinationRealTransform(t));
			transforms.add(t);
			weights.add(1.0);
//			add( t );
		}
	}

	private LinearCombinationRealTransform( List<RealTransform> ts, List<Double> ws) { 
		transforms = new ArrayList<>();
		transforms.addAll(ts);

		weights = new ArrayList<>();
		weights.addAll(ws);
		this.w = 1.0;
	}

	@Override
	public int numSourceDimensions() {
		return nd;
	}

	@Override
	public int numTargetDimensions() {
		return nd;
	}

	@Override
	public void apply(double[] source, double[] target) {
		if( transforms.size() == 0 && source != target )
		{
			System.arraycopy(source, 0, target, 0, nd);
		}
		else
		{
			Arrays.fill(target, 0);
			
			double ww = 1;
			for( int i = 0; i < transforms.size(); i++ )
			{
				ww = weights.get(i);
//				ww = w * weights.get(i);
//				WeightedRealTransfom xfm = transforms.get(i);
//				LinearCombinationRealTransfom xfm = transforms.get(i);
				RealTransform xfm = transforms.get(i);
				xfm.apply(source, tmp);
				System.out.println( "  lincombo " + i + " : " + Arrays.toString(target));
				for( int d = 0; d < nd; d++ )
					target[ d ] += ww * tmp[ d ];
			}
		}
	}

	@Override
	public void apply(RealLocalizable source, RealPositionable target) {
		if( transforms.size() == 0 && source != target )
		{
			target.setPosition(source);
		}
		else
		{
			for (int d = 0; d < nd; d++)
				accumulator.setPosition(0.0, d);

			double ww = 1;
			for( int i = 0; i < transforms.size(); i++ )
			{
				ww = weights.get(i);
//				ww = w * weights.get(i);
//				WeightedRealTransfom xfm = transforms.get(i);
//				LinearCombinationRealTransfom xfm = transforms.get(i);
				RealTransform xfm = transforms.get(i);
				xfm.apply(source, tmpPt);
				System.out.println( "  lincombo " + i + " : " + accumulator );
				for (int d = 0; d < nd; d++)
					accumulator.setPosition( 
							accumulator.getDoublePosition(d) + 
							ww * tmpPt.getDoublePosition(d), d);
			}

			target.setPosition(accumulator);
		}
	}
	
	public void set( final LinearCombinationRealTransform c )
	{
//		w = 1.0;
//		transforms.clear();
//		weights.clear();
//		add( c );

		w = c.w; 

		transforms.clear();
		transforms.addAll( c.transforms );

		weights.clear();
		weights.addAll(c.weights);
	}

//	public void addV1( final RealTransform c )
//	{
//		if( nd > 0 && ( c.numSourceDimensions() != nd  || c.numTargetDimensions() != nd  ))
//			return; // do nothing I guess
//		
//		if( c instanceof WeightedRealTransfom )
//			transforms.add((WeightedRealTransfom)c);
//		else
//			transforms.add(new WeightedRealTransfom(c));
//	}
	
	@Override
	public void add( final LinearCombinationRealTransform c )
	{
		if( nd > 0 && ( c.numSourceDimensions() != nd  || c.numTargetDimensions() != nd  ))
			return; // do nothing I guess
		
//		if( c instanceof LinearCombinationRealTransfom )
//			transforms.add((LinearCombinationRealTransfom)c);
//		else
//			transforms.add(new LinearCombinationRealTransfom(c));

		transforms.add( c );
		weights.add(1.0);
	}

	@Override
	public LinearCombinationRealTransform copy() {
//		return new LinearCombinationRealTransform( transforms);
		return new LinearCombinationRealTransform( transforms, weights );
	}
	
	@Override
	public void mul( final float c )
	{
		setReal( getRealDouble() * c );
	}

	@Override
	public void mul( final double c )
	{
		setReal( getRealDouble() * c );
	}	

	@Override
	public double getMaxValue()
	{
		return Double.MAX_VALUE;
	}

	@Override
	public double getMinValue()
	{
		return -Double.MAX_VALUE;
	}

	@Override
	public double getMinIncrement()
	{
		return Double.MIN_VALUE;
	}

	@Override
	public int getBitsPerPixel()
	{
		return 64;
	}

	@Override
	public double getRealDouble() {
		return 1.0;
	}

	@Override
	public float getRealFloat() {
		return 1.0f;
	}

	@Override
	public void setReal(float f) {
		for( int i = 0; i < weights.size(); i++ )
			weights.set(i, weights.get(i) * (f / w));

		this.w = f;
	}

	@Override
	public void setReal(double f) {
		for( int i = 0; i < weights.size(); i++ )
			weights.set(i, weights.get(i) * (f / w));

		this.w = f;
	}

	@Override
	public LinearCombinationRealTransform createVariable() {
		return new LinearCombinationRealTransform( nd );
	}

}
