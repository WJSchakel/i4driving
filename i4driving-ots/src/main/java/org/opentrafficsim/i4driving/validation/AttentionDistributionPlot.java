package org.opentrafficsim.i4driving.validation;

import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.stream.IntStream;

import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Time;
import org.djutils.event.Event;
import org.djutils.event.EventListener;
import org.djutils.means.ArithmeticMean;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.LegendItem;
import org.jfree.chart.LegendItemCollection;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.event.AxisChangeEvent;
import org.jfree.chart.event.AxisChangeListener;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StackedXYAreaRenderer2;
import org.jfree.chart.ui.RectangleEdge;
import org.jfree.data.DomainOrder;
import org.jfree.data.xy.TableXYDataset;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.draw.graphs.AbstractBoundedPlot;
import org.opentrafficsim.draw.graphs.GraphType;
import org.opentrafficsim.draw.graphs.PlotScheduler;
import org.opentrafficsim.i4driving.tactical.perception.mental.channel.ChannelFuller;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;

public class AttentionDistributionPlot extends AbstractBoundedPlot implements TableXYDataset, EventListener
{

    /** */
    private static final long serialVersionUID = 20251127L;

    private final SortedMap<Double, ArithmeticMean<Double, Double>[]> yValues = new TreeMap<>();

    /** Sorted X-values in order to couple them to an index. */
    private final List<Double> xValues = new ArrayList<>();

    private final AttentionContext context;

    private LaneBasedGtu gtu;

    private Length zeroOdometer;

    public AttentionDistributionPlot(final AttentionContext context)
    {
        super(PlotScheduler.OFFLINE, "Attention Distribution", Duration.instantiateSI(0.5), Duration.ZERO);
        this.context = context;
        setChart(createChart());
        context.getNetwork().addListener(this, Network.GTU_ADD_EVENT);
    }

    private JFreeChart createChart()
    {
        NumberAxis xAxis = new NumberAxis("Distance to intersection [m]");
        NumberAxis yAxis = new NumberAxis("Attention [-]");
        StackedXYAreaRenderer2 renderer = new StackedXYAreaRenderer2(); // 2 deals correctly with negative values
        XYPlot plot = new XYPlot(this, xAxis, yAxis, renderer);
        LegendItemCollection legend = new LegendItemCollection();
        IntStream.range(0, this.context.getNumberOfGroups())
                .forEach((i) -> legend.add(new LegendItem(this.context.getGroupLabel(this.context.getNumberOfGroups() - i - 1),
                        this.context.getGroupColor(this.context.getNumberOfGroups() - i - 1))));
        legend.add(new LegendItem("Idle", this.context.getIdleColor()));
        legend.add(new LegendItem("Anticipation Reliance", this.context.getAnticipationRelianceColor()));
        plot.setFixedLegendItems(legend);
        renderer.setSeriesPaint(0, this.context.getAnticipationRelianceColor());
        renderer.setSeriesPaint(1, this.context.getIdleColor());
        IntStream.range(0, this.context.getNumberOfGroups())
                .forEach((i) -> renderer.setSeriesPaint(i + 2, AttentionDistributionPlot.this.context.getGroupColor(i)));
        JFreeChart chart = new JFreeChart(getCaption(), JFreeChart.DEFAULT_TITLE_FONT, plot, true);
        chart.getLegend().setPosition(RectangleEdge.RIGHT);
        return chart;
    }

    @Override
    public int getSeriesCount()
    {
        return this.context.getNumberOfGroups() + 2;
    }

    @Override
    public Comparable<?> getSeriesKey(final int series)
    {
        return series == 0 ? "Anticipation Reliance" : (series == 1 ? "Idle" : this.context.getGroupLabel(series - 2));
    }

    @Override
    public int indexOf(final Comparable seriesKey)
    {
        return "Anticipation Reliance".equals(seriesKey) ? 0
                : ("Idle".equals(seriesKey) ? 1 : this.context.getGroupIndexByLabel((String) seriesKey));
    }

    @Override
    public DomainOrder getDomainOrder()
    {
        return DomainOrder.ASCENDING;
    }

    @Override
    public int getItemCount()
    {
        return this.yValues.size();
    }

    @Override
    public synchronized int getItemCount(final int series)
    {
        return getItemCount();
    }

    @Override
    public synchronized Number getX(final int series, final int item)
    {
        return this.xValues.get(item) + 0.5 * this.context.getBinSize();
    }

    @Override
    public double getXValue(final int series, final int item)
    {
        return getX(series, item).doubleValue();
    }

    @Override
    public synchronized Number getY(final int series, final int item)
    {
        return this.yValues.get(this.xValues.get(item))[series].getMean();
    }

    @Override
    public double getYValue(final int series, final int item)
    {
        return getY(series, item).doubleValue();
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
        //
    }

    @Override
    @SuppressWarnings("unchecked")
    public void notify(final Event event) throws RemoteException
    {
        if (event.getType().equals(Network.GTU_ADD_EVENT))
        {
            if (this.context.getGtuId().equals(event.getContent()))
            {
                this.gtu = (LaneBasedGtu) this.context.getNetwork().getGTU(this.context.getGtuId());
                this.gtu.addListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
                this.context.getNetwork().removeListener(this, Network.GTU_ADD_EVENT);
            }
        }
        else if (event.getType().equals(LaneBasedGtu.LANEBASED_MOVE_EVENT))
        {
            // Get x coordinate
            if (this.zeroOdometer == null)
            {
                this.zeroOdometer = this.context.getZeroOdometer(this.gtu);
            }
            Length relativeOdometer = this.zeroOdometer.minus(this.gtu.getOdometer());
            if (relativeOdometer.si < -this.context.getMaxDistance(this.gtu).si)
            {
                this.gtu.removeListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
                return;
            }
            int n = (int) Math.floor(relativeOdometer.si / this.context.getBinSize());
            double itemX = n * this.context.getBinSize();
            if (!this.xValues.contains(itemX))
            {
                synchronized (this)
                {
                    Optional<Double> index = this.xValues.stream().filter((v) -> v > itemX).findFirst();
                    if (index.isPresent())
                    {
                        this.xValues.add(this.xValues.indexOf(index.get()), itemX);
                    }
                    else
                    {
                        this.xValues.add(itemX);
                    }
                    ArithmeticMean<Double, Double>[] means = new ArithmeticMean[getSeriesCount()];
                    IntStream.range(0, getSeriesCount()).forEach((i) -> means[i] = new ArithmeticMean<>());
                    this.yValues.put(itemX, means);
                }
            }

            // Get attention levels
            ChannelFuller fuller = (ChannelFuller) this.gtu.getTacticalPlanner().getPerception().getMental();
            double sumAttention = 0.0;
            Map<Integer, Double> groupAttention = new LinkedHashMap<>();
            IntStream.range(0, this.context.getNumberOfGroups()).forEach((i) -> groupAttention.put(i, 0.0));
            for (Object channel : fuller.getChannels())
            {
                double attention = fuller.getAttention(channel);
                sumAttention += attention;
                int groupIndex = this.context.getGroupIndex(channel);
                if (groupIndex >= 0)
                {
                    groupAttention.compute(groupIndex, (i, v) -> v + attention);
                }
            }
            double anticipationReliance = this.gtu.getParameters().getParameterOrNull(Fuller.TS).doubleValue()
                    * this.gtu.getParameters().getParameterOrNull(Fuller.TC).doubleValue() - 1.0;

            // Add attention values to cumulative means
            ArithmeticMean<Double, Double>[] means = this.yValues.get(itemX);
            means[0].add(Math.min(0.0, -anticipationReliance));
            means[1].add(1.0 - sumAttention); // idle
            for (Entry<Integer, Double> entry : groupAttention.entrySet())
            {
                means[entry.getKey() + 2].add(entry.getValue());
            }

            notifyPlotChange();
            setAutoBoundDomain(getChart().getXYPlot());
            
        }

    }

}
