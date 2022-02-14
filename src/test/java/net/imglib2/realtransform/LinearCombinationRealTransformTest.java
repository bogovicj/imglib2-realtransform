package net.imglib2.realtransform;

import java.util.ArrayList;
import java.util.List;

import net.imglib2.RandomAccess;
import net.imglib2.RealPoint;
import net.imglib2.RealRandomAccess;
import net.imglib2.RealRandomAccessible;
import net.imglib2.img.list.ListImg;
import net.imglib2.interpolation.randomaccess.NLinearInterpolatorFactory;
import net.imglib2.realtransform.interpolation.LinearCombinationRealTransform;
import net.imglib2.realtransform.interpolation.WeightedRealTransfom;
import net.imglib2.view.ExtendedRandomAccessibleInterval;
import net.imglib2.view.Views;

public class LinearCombinationRealTransformTest {

	public static void main(String[] args) {
		
//		linearComboTest();
		linearComboInterpTest();
	}

	public static void linearComboInterpTest() {

		LinearCombinationRealTransform s2w = new LinearCombinationRealTransform(new Scale(2.0));
		LinearCombinationRealTransform s4w = new LinearCombinationRealTransform(new Scale(4.0));
		
		ArrayList<LinearCombinationRealTransform> list = new ArrayList<LinearCombinationRealTransform>();
		list.add(s2w);
		list.add(s4w);

		ListImg<LinearCombinationRealTransform> xfmImg = new ListImg<>( list, 2 );
		
		ExtendedRandomAccessibleInterval<LinearCombinationRealTransform, ListImg<LinearCombinationRealTransform>> imgext = Views.extendBorder(xfmImg);
		RealRandomAccessible<LinearCombinationRealTransform> interpxfm = Views.interpolate( imgext,
				new NLinearInterpolatorFactory<>());
		
		RandomAccess<LinearCombinationRealTransform> ra = xfmImg.randomAccess();
		RandomAccess<LinearCombinationRealTransform> xra = imgext.randomAccess();
		RealRandomAccess<LinearCombinationRealTransform> rra = interpxfm.realRandomAccess();

//		ra.setPosition(1, 0);
//		LinearCombinationRealTransform xfm = ra.get();

//		xra.setPosition(1, 0);
//		LinearCombinationRealTransform xfm = xra.get();

//		rra.setPosition(1.0, 0);
		rra.setPosition(0.5, 0);
		LinearCombinationRealTransform xfm = rra.get();
		
		RealPoint p = new RealPoint( 10.0 );
		RealPoint q = new RealPoint( 0.0 );

		xfm.apply(p, q);
		
		System.out.println( q );

	}
	
	public static void linearComboTest() {

		Scale s2 = new Scale( 2.0 );
		Scale s4 = new Scale( 4.0 );

		LinearCombinationRealTransform st = new LinearCombinationRealTransform( 1 );

		LinearCombinationRealTransform s2w = new LinearCombinationRealTransform( s2 );
		LinearCombinationRealTransform s4w = new LinearCombinationRealTransform( s4 );

		s2w.mul(0.5);
		s4w.mul(0.5);
		
		st.add(s2w);
		st.add(s4w);
		
		final RealPoint p = new RealPoint( 10.0 );
		final RealPoint q = new RealPoint( 0.0 );
		st.apply(p, q);
		System.out.println( q );
	}

	public static void weightedAndLinearComboTest() {

//		Scale s2 = new Scale( 2.0 );
//		Scale s4 = new Scale( 4.0 );
//		Translation t = new Translation( -11.0 );
//
//		LinearCombinationRealTransform st = new LinearCombinationRealTransform( 1 );
//
////		st.add( new WeightedRealTransfom(s2, 0.5));
////		st.add( new WeightedRealTransfom(s4, 0.5));
//
//		WeightedRealTransfom s2w = new WeightedRealTransfom(s2);
//		WeightedRealTransfom s4w = new WeightedRealTransfom(s4);
//		
//		s2w.mul(0.5);
//		s4w.mul(0.5);
//		
//		st.add(s2w);
//		st.add(s4w);
//		
//		final RealPoint p = new RealPoint( 10.0 );
//		final RealPoint q = new RealPoint( 0.0 );
//		st.apply(p, q);
//		System.out.println( q );
	}

}
