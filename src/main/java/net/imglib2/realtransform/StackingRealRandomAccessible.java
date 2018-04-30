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

import net.imglib2.Localizable;
import net.imglib2.RealInterval;
import net.imglib2.RealLocalizable;
import net.imglib2.RealRandomAccess;
import net.imglib2.RealRandomAccessible;

/**
 * A {@link RealRandomAccessible} whose samples are generated by trivial
 * projection of the lower dimensional {@link RealRandomAccessible source}
 * into a higher dimensional {@link RealRandomAccessible target} by continuous
 * stacking, i.e. when projecting from 1D to 3D, the samples at
 * f(5) = g(5,2,3) = g(5,1,6) = g(5,18,-20)...
 *
 * @author Stephan Saalfeld
 */
public class StackingRealRandomAccessible< T > implements RealRandomAccessible< T >
{
	/**
	 * {@link RealRandomAccess} that generates its samples from a lower
	 * dimensional {@link RealRandomAccess source}
	 *
	 */
	public class StackingRealRandomAccess implements RealRandomAccess< T >
	{
		final protected RealRandomAccess< T > sourceAccess;

		final double[] position;
		final float[] fMove;
		final double[] dMove;
		final int[] iMove;
		final long[] lMove;

		public StackingRealRandomAccess()
		{
			sourceAccess = source.realRandomAccess();
			fMove = new float[ sourceAccess.numDimensions() ];
			dMove = new double[ fMove.length ];
			iMove = new int[ fMove.length ];
			lMove = new long[ fMove.length ];
			position = new double[ numDimensions ];
		}

		@Override
		public void move( final float distance, final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.move( distance, d );
			else
				position[ d ] += distance;
		}

		@Override
		public void move( final double distance, final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.move( distance, d );
			else
				position[ d ] += distance;
		}

		@Override
		public void move( final RealLocalizable localizable )
		{
			for ( int d = 0; d < sourceNumDimensions; ++d )
				dMove[ d ] = localizable.getDoublePosition( d );
			sourceAccess.move( dMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] += localizable.getDoublePosition( d );
		}

		@Override
		public void move( final float[] distance )
		{
			System.arraycopy( distance, 0, fMove, 0, sourceNumDimensions );
			sourceAccess.move( fMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] += distance[ d ];
		}

		@Override
		public void move( final double[] distance )
		{
			System.arraycopy( distance, 0, dMove, 0, sourceNumDimensions );
			sourceAccess.move( dMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] += distance[ d ];
		}

		@Override
		public void setPosition( final RealLocalizable localizable )
		{
			for ( int d = 0; d < sourceNumDimensions; ++d )
				dMove[ d ] = localizable.getDoublePosition( d );
			sourceAccess.setPosition( dMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] = localizable.getDoublePosition( d );
		}

		@Override
		public void setPosition( final float[] pos )
		{
			System.arraycopy( pos, 0, fMove, 0, sourceNumDimensions );
			sourceAccess.setPosition( fMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] = pos[ d ];
		}

		@Override
		public void setPosition( final double[] pos )
		{
			System.arraycopy( pos, 0, dMove, 0, sourceNumDimensions );
			sourceAccess.setPosition( dMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] = pos[ d ];
		}

		@Override
		public void setPosition( final float pos, final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.setPosition( pos, d );
			else
				position[ d ] = pos;
		}

		@Override
		public void setPosition( final double pos, final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.setPosition( pos, d );
			else
				position[ d ] = pos;
		}

		@Override
		public void fwd( final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.fwd( d );
			else
				++position[ d ];
		}

		@Override
		public void bck( final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.bck( d );
			else
				--position[ d ];
		}

		@Override
		public void move( final int distance, final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.move( distance, d );
			else
				position[ d ] += distance;
		}

		@Override
		public void move( final long distance, final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.move( distance, d );
			else
				position[ d ] += distance;
		}

		@Override
		public void move( final Localizable localizable )
		{
			for ( int d = 0; d < sourceNumDimensions; ++d )
				lMove[ d ] = localizable.getLongPosition( d );
			sourceAccess.move( lMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] += localizable.getLongPosition( d );
		}

		@Override
		public void move( final int[] distance )
		{
			System.arraycopy( distance, 0, iMove, 0, sourceNumDimensions );
			sourceAccess.move( iMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] += distance[ d ];
		}

		@Override
		public void move( final long[] distance )
		{
			System.arraycopy( distance, 0, lMove, 0, sourceNumDimensions );
			sourceAccess.move( lMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] += distance[ d ];
		}

		@Override
		public void setPosition( final Localizable localizable )
		{
			for ( int d = 0; d < sourceNumDimensions; ++d )
				lMove[ d ] = localizable.getLongPosition( d );
			sourceAccess.setPosition( lMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] = localizable.getLongPosition( d );
		}

		@Override
		public void setPosition( final int[] pos )
		{
			System.arraycopy( pos, 0, iMove, 0, sourceNumDimensions );
			sourceAccess.setPosition( iMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] = pos[ d ];
		}

		@Override
		public void setPosition( final long[] pos )
		{
			System.arraycopy( pos, 0, lMove, 0, sourceNumDimensions );
			sourceAccess.setPosition( lMove );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				position[ d ] = pos[ d ];
		}

		@Override
		public void setPosition( final int pos, final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.setPosition( pos, d );
			else
				position[ d ] = pos;
		}

		@Override
		public void setPosition( final long pos, final int d )
		{
			if ( d < sourceNumDimensions )
				sourceAccess.setPosition( pos, d );
			else
				position[ d ] = pos;
		}

		@Override
		public T get()
		{
			return sourceAccess.get();
		}

		@Override
		public StackingRealRandomAccess copy()
		{
			return new StackingRealRandomAccess();
		}

		@Override
		public StackingRealRandomAccess copyRealRandomAccess()
		{
			return copy();
		}

		@Override
		public void localize( float[] pos )
		{
			for ( int d = 0; d < sourceNumDimensions; ++d )
				pos[ d ] = sourceAccess.getFloatPosition( d );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				pos[ d ] = ( float )this.position[ d ];
		}

		@Override
		public void localize( double[] pos )
		{
			for ( int d = 0; d < sourceNumDimensions; ++d )
				pos[ d ] = sourceAccess.getDoublePosition( d );
			for ( int d = sourceNumDimensions; d < numDimensions; ++d )
				pos[ d ] = this.position[ d ];
		}

		@Override
		public float getFloatPosition( int d )
		{
			if ( d < sourceNumDimensions )
				return sourceAccess.getFloatPosition( d );
			return ( float )position[ d ];
		}

		@Override
		public double getDoublePosition( int d )
		{
			if ( d < sourceNumDimensions )
				return sourceAccess.getDoublePosition( d );
			return ( float )position[ d ];
		}

		@Override
		public int numDimensions()
		{
			return numDimensions;
		}
	}

	final protected int numDimensions;
	final protected int sourceNumDimensions;

	final protected RealRandomAccessible< T > source;

	public StackingRealRandomAccessible( final RealRandomAccessible< T > source, final int numDimensions )
	{
		this.source = source;
		sourceNumDimensions = source.numDimensions();
		this.numDimensions = sourceNumDimensions + numDimensions;
	}

	@Override
	public StackingRealRandomAccess realRandomAccess()
	{
		return new StackingRealRandomAccess();
	}

	/**
	 * To be overridden for {@link RealTransform} that can estimate the
	 * boundaries of a transferred {@link RealInterval}.
	 */
	@Override
	public StackingRealRandomAccess realRandomAccess( final RealInterval interval )
	{
		return realRandomAccess();
	}

	@Override
	public int numDimensions()
	{
		return numDimensions;
	}
}
