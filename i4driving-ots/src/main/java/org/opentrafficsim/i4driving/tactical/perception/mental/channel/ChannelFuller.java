package org.opentrafficsim.i4driving.tactical.perception.mental.channel;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Function;

import org.djunits.value.vdouble.scalar.Duration;
import org.djutils.exceptions.Throw;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.ParameterTypeDuration;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.base.parameters.constraint.DualBound;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Estimation;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller.BehavioralAdaptation;

/**
 * Fuller implementation with perception channels. This is based on a set of task suppliers, which may either provide static
 * tasks (always the same) or a dynamic set of tasks (e.g. per conflicting road present). When relevant, task suppliers need to
 * map objects to channel keys when they are invoked to return the currently applicable channel tasks. For example mapping a
 * single conflict to a common key that refers to a channel based on a group of conflicts. In this way the correct perception
 * delay can be found when only knowing the single conflict, without knowing how it was grouped or what then defines the key.
 * @author wjschakel
 */
public class ChannelFuller implements ChannelMental
{

    /** Task capability in nominal task capability units, i.e. mean is 1. */
    public static final ParameterTypeDouble TC = Fuller.TC;

    /** Task saturation. */
    public static final ParameterTypeDouble TS = Fuller.TS;

    /** Critical task saturation. */
    public static final ParameterTypeDouble TS_CRIT = Fuller.TS_CRIT;

    /** Erroneous estimation factor on distance and speed difference. */
    public static final ParameterTypeDouble EST_FACTOR = new ParameterTypeDouble("f_est",
            "Erroneous estimation factor on distance and speed difference.", 1.0, NumericConstraint.POSITIVE);

    /** Level of attention, which is the maximum in the steady state of the Attention Matrix. */
    public static final ParameterTypeDouble ATT =
            new ParameterTypeDouble("ATT", "Attention (maximum of all channels).", 0.0, DualBound.UNITINTERVAL);

    /** Minimum perception delay. */
    public static final ParameterTypeDuration TAU_MIN = new ParameterTypeDuration("tau_min", "Minimum perception delay",
            Duration.instantiateSI(0.32), NumericConstraint.POSITIVEZERO)
    {
        /** */
        private static final long serialVersionUID = 20240919L;

        /** {@inheritDoc} */
        @Override
        public void check(final Duration value, final Parameters params) throws ParameterException
        {
            Throw.when(params.contains(TAU_MAX) && params.getParameter(TAU_MAX).lt(value), ParameterException.class,
                    "Value of tau_max less smaller than tau_min.");

        }
    };

    /** Maximum perception delay. */
    public static final ParameterTypeDuration TAU_MAX = new ParameterTypeDuration("tau_max", "Maximum perception delay",
            Duration.instantiateSI(0.32 + 0.87), NumericConstraint.POSITIVE)
    {
        /** */
        private static final long serialVersionUID = 20240919L;

        /** {@inheritDoc} */
        @Override
        public void check(final Duration value, final Parameters params) throws ParameterException
        {
            Throw.when(params.contains(TAU_MIN) && params.getParameter(TAU_MIN).gt(value), ParameterException.class,
                    "Value of tau_min is greater than tau_max.");
        }
    };

    /** Task suppliers. */
    private Set<Function<LanePerception, Set<ChannelTask>>> taskSuppliers = new LinkedHashSet<>();

    /** Behavioral adaptations. */
    private Set<BehavioralAdaptation> behavioralAdapatations = new LinkedHashSet<>();

    /** Mappings from object to channel. */
    private Map<Object, Object> channelMapping = new LinkedHashMap<>();

    /** Stored perception delay per channel. */
    private Map<Object, Duration> perceptionDelay = new LinkedHashMap<>();

    /** Stored level of attention per channel. */
    private Map<Object, Double> attention = new LinkedHashMap<>();

    /**
     * Constructor.
     * @param taskSuppliers task suppliers.
     * @param behavioralAdapatations behavioral adaptations.
     */
    public ChannelFuller(final Collection<Function<LanePerception, Set<ChannelTask>>> taskSuppliers,
            final Set<BehavioralAdaptation> behavioralAdapatations)
    {
        this.taskSuppliers.addAll(taskSuppliers);
        this.behavioralAdapatations.addAll(behavioralAdapatations);
    }

    /** {@inheritDoc} */
    @Override
    public void apply(final LanePerception perception) throws ParameterException, GtuException
    {
        // Clear mappings
        this.channelMapping.clear();

        // Gather all channels and their maximum task demand
        Map<Object, Double> channelTaskDemand = new LinkedHashMap<>();
        for (Function<LanePerception, Set<ChannelTask>> taskFunction : this.taskSuppliers)
        {
            for (ChannelTask task : taskFunction.apply(perception)) // if applicable will (re)map objects to channel keys
            {
                double td = task.getDemand(perception);
                Throw.when(td >= 1.0, GtuException.class,
                        "Task %s produced task demand that is greater than, or equal to, 1.0.", task.getId());
                channelTaskDemand.merge(task.getChannel(), td, Math::max); // map to max value
            }
        }

        // Apply attention matrix and couple channel to indices
        double[] tdArray = new double[channelTaskDemand.size()];
        int index = 0;
        double sumTaskDemand = 0.0;
        Map<Object, Integer> channelIndex = new LinkedHashMap<>();
        for (Entry<Object, Double> entry : channelTaskDemand.entrySet())
        {
            channelIndex.put(entry.getKey(), index);
            double td = entry.getValue();
            tdArray[index] = td;
            sumTaskDemand += td;
            index++;
        }
        AttentionMatrix matrix = new AttentionMatrix(tdArray);

        // Determine attention and perception delay per channel
        double maxAttention = 0.0;
        this.perceptionDelay.clear();
        this.attention.clear();
        Parameters parameters = perception.getGtu().getParameters();
        Duration tauMin = parameters.getParameter(TAU_MIN);
        Duration tauMax = parameters.getParameter(TAU_MAX);
        double tc = parameters.getParameter(TC);
        for (Entry<Object, Integer> entry : channelIndex.entrySet())
        {
            index = entry.getValue();
            this.perceptionDelay.put(entry.getKey(),
                    Duration.interpolate(tauMin, tauMax, matrix.getDeterioration(index)).divide(tc));
            double att = matrix.getAttention(index);
            maxAttention = Double.max(maxAttention, att);
            this.attention.put(entry.getKey(), att);
        }

        // Calculate task saturation, perception errors, and apply behavioral adaptations
        double ts = sumTaskDemand / tc;
        double deltaCrit = parameters.contains(TS_CRIT) ? 1.0 - parameters.getParameter(TS_CRIT) : 0.0;
        parameters.setParameter(TS, ts);
        parameters.setParameter(EST_FACTOR,
                Math.pow(Math.max(ts + deltaCrit, 1.0), parameters.getParameter(Estimation.OVER_EST)));
        parameters.setParameter(ATT, maxAttention);
        for (BehavioralAdaptation behavioralAdapatation : this.behavioralAdapatations)
        {
            behavioralAdapatation.adapt(parameters, ts);
        }
    }

    /** {@inheritDoc} */
    @Override
    public Duration getPerceptionDelay(final Object obj)
    {
        return this.perceptionDelay.get(getChannel(obj));
    }

    /** {@inheritDoc} */
    @Override
    public double getAttention(final Object obj)
    {
        return this.attention.get(getChannel(obj));
    }

    /** {@inheritDoc} */
    @Override
    public void mapToChannel(final Object obj, final Object channel)
    {
        this.channelMapping.put(obj, channel);
    }

    /**
     * Returns the relevant channel key for the object. This is a channel key mapped to the object, or the object itself if
     * there is no such mapping (in which case the object should itself directly be a channel key).
     * @param obj object.
     * @return relevant channel key for the object.
     */
    private Object getChannel(final Object obj)
    {
        if (this.channelMapping.containsKey(obj))
        {
            return this.channelMapping.get(obj);
        }
        Throw.when(!this.perceptionDelay.containsKey(obj), IllegalArgumentException.class, "Channel %s is not present.", obj);
        return obj;
    }

    /**
     * Returns the current channels.
     * @return set of channels
     */
    public Set<Object> getChannels()
    {
        return new LinkedHashSet<>(this.attention.keySet());
    }

}
