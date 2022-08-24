package net.imglib2.benchmarks;

import java.util.concurrent.TimeUnit;

import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Fork;
import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.runner.Runner;
import org.openjdk.jmh.runner.RunnerException;
import org.openjdk.jmh.runner.options.Options;
import org.openjdk.jmh.runner.options.OptionsBuilder;
import org.openjdk.jmh.runner.options.TimeValue;

@State( Scope.Thread )
@Fork( 1 )
public class ConditionBenchmark {

	double[] del2, del3;
	double[] p2, p3, p4, q2, q3, q4;

	public static void main(String... args) throws RunnerException {

        Options opt = new OptionsBuilder()
                .include(ConditionBenchmark.class.getSimpleName())
                .forks(1)
				.warmupIterations( 4 )
				.measurementIterations( 8 )
				.warmupTime( TimeValue.milliseconds( 2000 ) )
				.measurementTime( TimeValue.milliseconds( 5000 ) )
                .build();

        new Runner(opt).run();
	}

	@Setup( Level.Iteration )
	public void allocate()
	{
		del2 = new double[]{ Math.random(), Math.random() };
		del3 = new double[]{ Math.random(), Math.random(), Math.random() };

		p2 = new double[]{ Math.random(), Math.random()  };
		p3 = new double[]{ Math.random(), Math.random(), Math.random() };
		p4 = new double[]{ Math.random(), Math.random(), Math.random(), Math.random() };

		q2 = new double[]{ Math.random(), Math.random()  };
		q3 = new double[]{ Math.random(), Math.random(), Math.random() };
		q4 = new double[]{ Math.random(), Math.random(), Math.random(), Math.random() };
	}

	@Benchmark
	@BenchmarkMode( Mode.AverageTime )
	@OutputTimeUnit( TimeUnit.NANOSECONDS )
	public void displacement2d()
	{
		displacement( p2, q2, del2 );
	}

	@Benchmark
	@BenchmarkMode( Mode.AverageTime )
	@OutputTimeUnit( TimeUnit.NANOSECONDS )
	public void displacement2dCondition()
	{
		displacementCondition( p2, q3, del2 );
	}

	@Benchmark
	@BenchmarkMode( Mode.AverageTime )
	@OutputTimeUnit( TimeUnit.NANOSECONDS )
	public void displacement2dCondition2()
	{
		displacementCondition( p3, q2, del2 );
	}

	@Benchmark
	@BenchmarkMode( Mode.AverageTime )
	@OutputTimeUnit( TimeUnit.NANOSECONDS )
	public void displacement3d()
	{
		displacement( p3, q3, del3 );
	}

	@Benchmark
	@BenchmarkMode( Mode.AverageTime )
	@OutputTimeUnit( TimeUnit.NANOSECONDS )
	public void displacement3dCondition()
	{
		displacementCondition( p3, q4, del3 );
	}

	@Benchmark
	@BenchmarkMode( Mode.AverageTime )
	@OutputTimeUnit( TimeUnit.NANOSECONDS )
	public void displacement3dCondition2()
	{
		displacementCondition( p4, q3, del3 );
	}

	public static void displacement( double[] p, double[] q, double[] del )
	{
		for( int i = 0; i < p.length; i++ )
			q[i] = p[i] + del[i];
	}

	public static void displacementCondition( double[] p, double[] q, double[] del )
	{
		final int N = q.length < p.length ? q.length : p.length;
		for( int i = 0; i < N; i++ )
			q[i] = p[i] + del[i];
	}

}
