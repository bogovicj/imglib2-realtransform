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

import net.imglib2.FinalInterval;
import net.imglib2.RandomAccess;
import net.imglib2.RandomAccessible;
import net.imglib2.RandomAccessibleInterval;
import net.imglib2.RealRandomAccess;
import net.imglib2.RealRandomAccessible;
import net.imglib2.interpolation.InterpolatorFactory;
import net.imglib2.view.Views;

/**
 * Convenience factory methods for {@link RealRandomAccessible
 * RealRandomAccessibles} transformed in real coordinate space by
 * {@link InvertibleRealTransform InvertibleRealTransforms}.
 *
 * @author Stephan Saalfeld <saalfelds@janelia.hhmi.org>
 */
public class RealViews
{
	final private static double[] fillScales( final double[] scales, final int n )
	{
		if ( scales.length == n )
			return scales;
		else
		{
			final double[] filledScales = new double[ n ];
			Arrays.fill( filledScales, 1 );
			System.arraycopy( scales, 0, filledScales, 0, Math.max( scales.length, n ) );
			return filledScales;
		}
	}

	/**
	 * See a {@link RealRandomAccessible} as transformed by an
	 * {@link InvertibleRealTransform}. The {@link InvertibleRealTransform} is
	 * interpreted according to the natural understanding that the source is
	 * transformed by it. E.g. a positive translation of dimension <em>x</em>
	 * would shift the source to the right. Therefore, the samples need to be
	 * generated by the inverse of the {@link InvertibleRealTransform}. Here,
	 * the inverse is realized by {@link InverseRealTransform}. That way,
	 * changing the state of the {@link InvertibleRealTransform} will
	 * immediately change the state of any new {@link RealRandomAccess}
	 * generated by the view.
	 *
	 * @param source
	 *            the {@link RealRandomAccessible} to be transformed
	 * @param transformFromSource
	 *            the {@link InvertibleRealTransform} transforming source
	 *            coordinates to coordinates of the returned
	 *            {@link RealRandomAccessible}
	 *
	 * @return {@link RealTransformRealRandomAccessible} representing the
	 *         transformed source
	 */
	public static < T > RealTransformRealRandomAccessible< T, InverseRealTransform > transformReal( final RealRandomAccessible< T > source, final InvertibleRealTransform transformFromSource )
	{
		return new RealTransformRealRandomAccessible< T, InverseRealTransform >( source, new InverseRealTransform( transformFromSource ) );
	}

	/**
	 * See a {@link RealRandomAccessible} as a {@link RandomAccessible}
	 * transformed by an {@link InvertibleRealTransform}. The
	 * {@link InvertibleRealTransform} is interpreted according to the natural
	 * understanding that the source is transformed by it. E.g. a positive
	 * translation of dimension <em>x</em> would shift the source to the right.
	 * Therefore, the samples need to be generated by the inverse of the
	 * {@link InvertibleRealTransform}. Here, the inverse is realized by
	 * {@link InverseRealTransform}. That way, changing the state of the
	 * {@link InvertibleRealTransform} will immediately change the state of any
	 * new {@link RandomAccess} generated by the view.
	 *
	 * @param source
	 *            the {@link RealRandomAccessible} to be transformed
	 * @param transformFromSource
	 *            the {@link InvertibleRealTransform} transforming source
	 *            coordinates to coordinates of the returned
	 *            {@link RealRandomAccessible}
	 *
	 * @return {@link RealTransformRandomAccessible} representing the
	 *         transformed source
	 */
	public static < T > RealTransformRandomAccessible< T, InverseRealTransform > transform( final RealRandomAccessible< T > source, final InvertibleRealTransform transformFromSource )
	{
		return new RealTransformRandomAccessible< T, InverseRealTransform >( source, new InverseRealTransform( transformFromSource ) );
	}

	/**
	 * See a {@link RealRandomAccessible} as transformed by an {@link AffineGet}
	 * . The {@link AffineGet} is interpreted according to the natural
	 * understanding that the source is transformed by it. E.g. a positive
	 * translation of dimension <em>x</em> would shift the source to the right.
	 * Therefore, the samples need to be generated by the inverse of the
	 * {@link AffineGet}. Here, the {@link AffineGet} is inverted using it's
	 * {@link AffineGet#inverse()} method that is expected to generate an
	 * inverse that changes with the original transformation accordingly. That
	 * way, changing the state of the {@link AffineGet} will immediately change
	 * the state of any new {@link RealRandomAccess} generated by the view.
	 *
	 * @param source
	 *            the {@link RealRandomAccessible} to be transformed
	 * @param transformFromSource
	 *            the {@link InvertibleRealTransform} transforming source
	 *            coordinates to coordinates of the returned
	 *            {@link RealRandomAccessible}
	 *
	 * @return {@link AffineRealRandomAccessible} representing the transformed
	 *         source
	 */
	public static < T > AffineRealRandomAccessible< T, AffineGet > affineReal( final RealRandomAccessible< T > source, final AffineGet transformFromSource )
	{
		return new AffineRealRandomAccessible< T, AffineGet >( source, transformFromSource.inverse() );
	}

	/**
	 * See a {@link RealRandomAccessible} as a {@link RandomAccessible}
	 * transformed by an {@link AffineGet}. The {@link AffineGet} is interpreted
	 * according to the natural understanding that the source is transformed by
	 * it. E.g. a positive translation of dimension <em>x</em> would shift the
	 * source to the right. Therefore, the samples need to be generated by the
	 * inverse of the {@link AffineGet}. Here, the {@link AffineGet} is inverted
	 * using it's {@link AffineGet#inverse()} method that is expected to
	 * generate and inverse that changes with the original transformation
	 * accordingly. That way, changing the state of the {@link AffineGet} will
	 * immediately change the state of any new {@link RandomAccess} generated
	 * by the view.
	 *
	 * @param source
	 *            the {@link RealRandomAccessible} to be transformed
	 * @param transformFromSource
	 *            the {@link InvertibleRealTransform} transforming source
	 *            coordinates to coordinates of the returned
	 *            {@link RealRandomAccessible}
	 *
	 * @return {@link AffineRandomAccessible} representing the transformed
	 *         source
	 */
	public static < T > AffineRandomAccessible< T, AffineGet > affine( final RealRandomAccessible< T > source, final AffineGet transformFromSource )
	{
		return new AffineRandomAccessible< T, AffineGet >( source, transformFromSource.inverse() );
	}

	/**
	 * See a {@link RealRandomAccessible} as scaled by a single scale factor.
	 * The scale factors are interpreted according to the natural
	 * understanding that the source is transformed by it.  E.g. a positive
	 * scale factor makes the source larger.  Therefore, the samples need to be
	 * generated by the inverse of the {@link Scale}.  Here, the
	 * {@link Scale} is inverted using it's {@link Scale#inverse()} method that
	 * is expected to generate an inverse that changes with the original
	 * transformation accordingly.  That way, changing the state of the
	 * {@link Scale} will immediately change the state of any new
	 * {@link RealRandomAccess} generated by the view.
	 *
	 * @param source
	 *            the {@link RealRandomAccessible} to be transformed
	 * @param scaleFromSource
	 *            the scale factor transforming coordinates to coordinates of
	 *            the returned {@link RealRandomAccessible}
	 *
	 * @return {@link RealRandomAccessible} representing the scaled source
	 */
	public static < T > RealRandomAccessible< T > scaleReal( final RealRandomAccessible< T > source, final double scaleFromSource )
	{
		final int n = source.numDimensions();
		final double[] scales = new double[ n ];
		Arrays.fill( scales, scaleFromSource );

		return scaleReal( source, scales );
	}

	/**
	 * See a {@link RealRandomAccessible} as scaled by per-dimension scale
	 * factors.  The scale factors are interpreted according to the natural
	 * understanding that the source is transformed by it.  E.g. a positive
	 * scale factor makes the source larger.  Therefore, the samples need to be
	 * generated by the inverse of the {@link Scale}.  Here, the
	 * {@link Scale} is inverted using it's {@link Scale#inverse()} method that
	 * is expected to generate an inverse that changes with the original
	 * transformation accordingly.  That way, changing the state of the
	 * {@link Scale} will immediately change the state of any new
	 * {@link RealRandomAccess} generated by the view.
	 *
	 * @param source
	 *            the {@link RealRandomAccessible} to be transformed
	 * @param scalesFromSource
	 *            the scale factors transforming coordinates to coordinates of
	 *            the returned {@link RealRandomAccessible}
	 *
	 * @return {@link RealRandomAccessible} representing the scaled source
	 */
	public static < T > RealRandomAccessible< T > scaleReal( final RealRandomAccessible< T > source, final double... scalesFromSource )
	{
		final int n = source.numDimensions();
		final double[] scales = fillScales( scalesFromSource, n );

		final AbstractScale scaleTransform;
		switch ( n )
		{
		case 2 :
			scaleTransform = new Scale2D( scales );
			break;
		case 3 :
			scaleTransform = new Scale3D( scales );
			break;
		default :
			scaleTransform = new Scale( scales );
		}

		/* TODO not ideal because scale is independent for all axes */
		return new AffineRealRandomAccessible< T, ScaleGet >( source, scaleTransform.inverse() );
	}

	/**
	 * See a {@link RealRandomAccessible} as scaled by a single scale factor
	 * and rastered.  The scale factors are interpreted according to the
	 * natural understanding that the source is transformed by it.  E.g. a
	 * positive scale factor makes the source larger.  Therefore, the samples
	 * need to be generated by the inverse of the {@link Scale}.  Here, the
	 * {@link Scale} is inverted using it's {@link Scale#inverse()} method that
	 * is expected to generate an inverse that changes with the original
	 * transformation accordingly.  That way, changing the state of the
	 * {@link Scale} will immediately change the state of any new
	 * {@link RealRandomAccess} generated by the view.
	 *
	 * @param source
	 *            the {@link RealRandomAccessible} to be transformed
	 * @param scaleFromSource
	 *            the scale factor transforming coordinates to coordinates of
	 *            the returned {@link RandomAccessible}
	 *
	 * @return {@link RandomAccessible} representing the scaled source
	 */
	public static < T > RandomAccessible< T > scale( final RealRandomAccessible< T > source, final double scaleFromSource )
	{
		final int n = source.numDimensions();
		final double[] scales = new double[ n ];
		Arrays.fill( scales, scaleFromSource );

		return scale( source, scales );
	}

	/**
	 * See a {@link RealRandomAccessible} as scaled by per-dimension scale
	 * factors and rastered.  The scale factors are interpreted according to
	 * the natural understanding that the source is transformed by it.  E.g. a
	 * positive scale factor makes the source larger.  Therefore, the samples
	 * need to be generated by the inverse of the {@link Scale}.  Here, the
	 * {@link Scale} is inverted using it's {@link Scale#inverse()} method that
	 * is expected to generate an inverse that changes with the original
	 * transformation accordingly.  That way, changing the state of the
	 * {@link Scale} will immediately change the state of any new
	 * {@link RealRandomAccess} generated by the view.
	 *
	 * @param source
	 *            the {@link RealRandomAccessible} to be transformed
	 * @param scalesFromSource
	 *            the scale factors transforming coordinates to coordinates of
	 *            the returned {@link RealRandomAccessible}
	 *
	 * @return {@link RandomAccessible} representing the scaled source
	 */
	public static < T > RandomAccessible< T > scale( final RealRandomAccessible< T > source, final double... scalesFromSource )
	{
		final int n = source.numDimensions();
		final double[] scales = fillScales( scalesFromSource, n );

		final AbstractScale scaleTransform;
		switch ( n )
		{
		case 2 :
			scaleTransform = new Scale2D( scales );
			break;
		case 3 :
			scaleTransform = new Scale3D( scales );
			break;
		default :
			scaleTransform = new Scale( scales );
		}

		/* TODO not ideal because scale is independent for all axes */
		return new AffineRandomAccessible< T, ScaleGet >( source, scaleTransform.inverse() );
	}

	/**
	 * See a {@link RandomAccessibleInterval} as scaled to the desired
	 * dimensions using a given {@link InterpolatorFactory}.
	 *
	 * @param source
	 *            the {@link RandomAccessibleInterval} to be scaled
	 * @param factory
	 *            the {@link InterpolatorFactory} that defines the interpolation
	 *            method when scaling.
	 * @param scalesFromSource
	 *            the scale factors transforming coordinates to coordinates of
	 *            the returned {@link RealRandomAccessible}
	 */
	public static < T > RandomAccessibleInterval< T > scale(
			final RandomAccessibleInterval< T > source,
			final InterpolatorFactory< T, RandomAccessible< T > > factory,
			final double... scalesFromSource )
	{
		final int n = source.numDimensions();
		final double[] scales = fillScales( scalesFromSource, n );

		final RealRandomAccessible< T > interpolant = Views.interpolate( Views.extendBorder( source ), factory );
		final RandomAccessible< T > scaled = scale( interpolant, scales );

		final long[] dimensions = new long[ source.numDimensions() ];
		for ( int d = 0; d < dimensions.length; ++d )
			dimensions[ d ] = ( long )( source.dimension( d ) * scales[ d ] );

		return Views.interval( scaled, new FinalInterval( dimensions ) );
	}
}
