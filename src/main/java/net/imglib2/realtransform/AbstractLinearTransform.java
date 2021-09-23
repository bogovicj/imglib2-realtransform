/*
 * #%L
 * ImgLib2: a general-purpose, multidimensional image processing library.
 * %%
 * Copyright (C) 2009 - 2020 Tobias Pietzsch, Stephan Preibisch, Stephan Saalfeld,
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

import net.imglib2.RealLocalizable;
import net.imglib2.RealPoint;
import net.imglib2.RealPositionable;

/**
 * An abstract implementation of a linear transformation that returns default
 * values referring to the identity transformation for all fields. It stores the matrix 
 * in an <em>m</em>&times;<em>n</em> array.
 * 
 * This implementation is not thread safe.  Create a {@link #copy()} for each
 * consumer.
 * 
 * @author John Bogovic
 * @author Stephan Saalfeld
 */
public abstract class AbstractLinearTransform implements LinearGet, AffineSet
{
	final protected int m;

	final protected int n;

	final protected double[][] matrix;

	final protected double[] tmp;

	final protected RealPoint[] ds;

	public AbstractLinearTransform( final double[][] matrix )
	{
		n = matrix.length;
		m = matrix[0].length;
		this.matrix = matrix;
		tmp = new double[ n ];
		ds = new RealPoint[ n ];

		for ( int r = 0; r < n; ++r )
		{
			ds[ r ] = new RealPoint( n );
		}
		updateDs();
	}

	public AbstractLinearTransform( final int m, final int n )
	{
		this.m = m;
		this.n = n;
		matrix = new double[ m ][ n ];
		tmp = new double[ n ];
		ds = new RealPoint[ n ];

		for ( int r = 0; r < n; ++r )
		{
			final RealPoint d = new RealPoint( n );
			matrix[r][r] = 1.0;
			d.setPosition( 1.0, r );
			ds[ r ] = d;
		}
	}

	protected void updateDs()
	{
		for ( int c = 0; c < n; ++c )
		{
			final RealPoint d = ds[ c ];
			for ( int r = 0; r < n; ++r )
				d.setPosition( matrix[r][c ], r );
		}
	}

	@Override
	public int numSourceDimensions()
	{
		return n;
	}

	@Override
	public int numTargetDimensions()
	{
		return m;
	}
	
	@Override
	public void apply( final double[] source, final double[] target )
	{
		assert source.length >= n && target.length >= m: "Source or target vector dimensions do not match with the transformation.";

		for ( int r = 0; r < m; ++r )
		{
			tmp[ r ] = 0;
			for ( int c = 0; c < n; ++c )
				tmp[ r ] += source[ c ] * matrix[r][c];
		}
		
		for ( int r = 0; r < m; ++r )
			target[ r ] = tmp[ r ];
	}
	
	@Override
	public void apply( final float[] source, final float[] target )
	{
		assert source.length >= n && target.length >= m: "Source or target vector dimensions do not match with the transformation.";

		for ( int r = 0; r < m; ++r )
		{
			tmp[ r ] = 0;
			for ( int c = 0; c < n; ++c )
				tmp[ r ] += source[ c ] * matrix[r][c];
		}

		for ( int r = 0; r < n; ++r )
			target[ r ] = ( float )( tmp[ r ] );
	}

	@Override
	public void apply( final RealLocalizable source, final RealPositionable target )
	{
		assert source.numDimensions() >= n && target.numDimensions() >= m: "Source or target vector dimensions do not match with the transformation.";

		for ( int r = 0; r < m; ++r )
		{
			tmp[ r ] = 0;
			for ( int c = 0; c < n; ++c )
				tmp[ r ] += source.getDoublePosition( c ) * matrix[r][c];
		}
		
		for ( int r = 0; r < m; ++r )
			target.setPosition( tmp[ r ] , r );
	}

	@Override
	public double get( final int row, final int column )
	{
		assert row >= 0 && row < m && column >= 0 && column <= n: "Row or column out of bounds.";

		return matrix[ row ][ column ];
	}

	@Override
	public double[] getRowPackedCopy()
	{
		final double[] copy = new double[ m * n ];
		for ( int r = 0, i = 0; r < m; ++r, ++i )
		{
			for ( int c = 0; c < n; ++c, ++i )
				copy[ i ] = matrix[r][c];
		}
		return copy;
	}

	@Override
	public RealLocalizable d( final int d )
	{
		assert d >= 0 && d < n: "Dimension out of bounds.";

		return ds[ d ];
	}
}
