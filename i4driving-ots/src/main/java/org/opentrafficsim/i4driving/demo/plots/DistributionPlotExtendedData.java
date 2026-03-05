package org.opentrafficsim.i4driving.demo.plots;

import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Time;
import org.djutils.exceptions.Throw;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYBarRenderer;
import org.jfree.data.DomainOrder;
import org.jfree.data.xy.IntervalXYDataset;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.draw.graphs.AbstractPlot;
import org.opentrafficsim.draw.graphs.GraphPath;
import org.opentrafficsim.draw.graphs.GraphPath.Section;
import org.opentrafficsim.draw.graphs.GraphType;
import org.opentrafficsim.kpi.interfaces.LaneData;
import org.opentrafficsim.kpi.sampling.SamplerData;
import org.opentrafficsim.kpi.sampling.SamplingException;
import org.opentrafficsim.kpi.sampling.Trajectory;
import org.opentrafficsim.kpi.sampling.data.ExtendedDataType;
import org.opentrafficsim.swing.graphs.OtsPlotScheduler;

/**
 * Distribution plot for extended data.
 * @author wjschakel
 */
public class DistributionPlotExtendedData extends AbstractPlot implements IntervalXYDataset
{

    /** Sampler data. */
    private final SamplerData<?> samplerData;

    /** KPI lane directions registered in the sampler. */
    private final GraphPath<? extends LaneData<?>> path;

    /** Data type. */
    private final ExtendedDataType<? extends Number, ?, ?, ?> dataType;

    /** Time of most recent update. */
    private double lastUpdateTime = Double.NEGATIVE_INFINITY;

    /** X-values. */
    private final double[] x;

    /** Y-values. */
    private final int[] y;

    /**
     * Constructor.
     * @param samplerData sampler data
     * @param path path
     * @param dataType data type
     * @param caption caption
     * @param xLabel label on x-axis
     * @param simulator simulator
     * @param xMin minimum x-value
     * @param xStep step value
     * @param xMax maximum x-value
     */
    public DistributionPlotExtendedData(final SamplerData<?> samplerData, final GraphPath<? extends LaneData<?>> path,
            final ExtendedDataType<? extends Number, ?, ?, ?> dataType, final String caption, final String xLabel,
            final OtsSimulatorInterface simulator, final double xMin, final double xStep, final double xMax)
    {
        super(new OtsPlotScheduler(simulator), caption, Duration.instantiateSI(10.0), Duration.ZERO);
        Throw.when(xMax <= xMin, IllegalArgumentException.class, "xMax must be greater than xMin");
        int n = (int) ((xMax - xMin + xStep / 1e9) / xStep) + 1;
        this.x = new double[n];
        for (int i = 0; i < n; i++)
        {
            this.x[i] = xMin + i * xStep;
        }
        this.y = new int[n];
        this.samplerData = samplerData;
        this.path = path;
        this.dataType = dataType;
        setChart(createChart(xLabel));
    }

    /**
     * Create a chart.
     * @param xLabel label on x-axis
     * @return JFreeChart; chart
     */
    private JFreeChart createChart(final String xLabel)
    {
        NumberAxis xAxis = new NumberAxis(xLabel);
        xAxis.setRange(this.x[0], this.x[this.x.length - 1]);
        NumberAxis yAxis = new NumberAxis("Count [-]");
        yAxis.setAutoRangeIncludesZero(true);
        XYBarRenderer renderer = new XYBarRenderer();
        XYPlot plot = new XYPlot(this, xAxis, yAxis, renderer);
        return new JFreeChart(getCaption(), JFreeChart.DEFAULT_TITLE_FONT, plot, false);
    }

    @Override
    public int getSeriesCount()
    {
        return 1;
    }

    @SuppressWarnings("rawtypes")
    @Override
    public Comparable getSeriesKey(final int series)
    {
        return Integer.valueOf(1);
    }

    @SuppressWarnings("rawtypes")
    @Override
    public int indexOf(final Comparable seriesKey)
    {
        return 0;
    }

    @Override
    public DomainOrder getDomainOrder()
    {
        return DomainOrder.ASCENDING;
    }

    @Override
    public int getItemCount(final int series)
    {
        return this.x.length - 1;
    }

    @Override
    public Number getX(final int series, final int item)
    {
        return this.x[item];
    }

    @Override
    public double getXValue(final int series, final int item)
    {
        return this.x[item];
    }

    @Override
    public Number getY(final int series, final int item)
    {
        return this.y[item];
    }

    @Override
    public double getYValue(final int series, final int item)
    {
        return this.y[item];
    }

    @Override
    public Number getStartX(final int series, final int item)
    {
        return this.x[item];
    }

    @Override
    public double getStartXValue(final int series, final int item)
    {
        return this.x[item];
    }

    @Override
    public Number getEndX(final int series, final int item)
    {
        return this.x[item + 1];
    }

    @Override
    public double getEndXValue(final int series, final int item)
    {
        return this.x[item + 1];
    }

    @Override
    public Number getStartY(final int series, final int item)
    {
        return getYValue(series, item);
    }

    @Override
    public double getStartYValue(final int series, final int item)
    {
        return getYValue(series, item);
    }

    @Override
    public Number getEndY(final int series, final int item)
    {
        return getYValue(series, item);
    }

    @Override
    public double getEndYValue(final int series, final int item)
    {
        return getYValue(series, item);
    }

    @Override
    public GraphType getGraphType()
    {
        return GraphType.OTHER;
    }

    @Override
    public String getStatusLabel(final double domainValue, final double rangeValue)
    {
        return " ";
    }

    @Override
    protected void increaseTime(final Time time)
    {
        if (this.path == null)
        {
            return; // initializing
        }
        try
        {
            double dx = this.x[1] - this.x[0];
            for (Section<? extends LaneData<?>> section : this.path.getSections())
            {
                for (LaneData<?> lane : section.sections())
                {
                    for (Trajectory<?> trajectory : this.samplerData.getTrajectoryGroup(lane))
                    {
                        int n = trajectory.size() - 1;
                        while (trajectory.getT(n) > this.lastUpdateTime && n >= 0)
                        {
                            double value = trajectory.getExtendedData(this.dataType, n).doubleValue();
                            if (!Double.isNaN(value))
                            {
                                int index = (int) Math.floor((value - this.x[0]) / dx);
                                if (0 <= index && index < this.y.length)
                                {
                                    this.y[index]++;
                                }
                            }
                            n--;
                        }
                    }
                }
            }
        }
        catch (SamplingException exception)
        {
            //
        }
        this.lastUpdateTime = time.si;
    }

}
