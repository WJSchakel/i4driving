package org.opentrafficsim.i4driving.tactical.perception.mental.channel;

import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.UUID;
import java.util.function.Function;

import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djutils.exceptions.Try;
import org.djutils.immutablecollections.ImmutableSet;
import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.ParameterTypeDuration;
import org.opentrafficsim.base.parameters.ParameterTypeLength;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.constraint.DualBound;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.i4driving.tactical.VisibilityLanePerception;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable.UnderlyingDistance;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.IntersectionPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.network.lane.conflict.Conflict;

/**
 * The intersection channel task looks at an intersection in an integrated form by grouping conflicts per conflicting road where
 * each conflicting road pertains to a perception channel. The ego distance towards the first conflict (crossing or merge) on
 * the intersection is a primary factor in the level of task demand in each channel. Time-proximity of a conflicting vehicle is
 * a secondary contributor to task demand in each channel.
 * @implNote There is much duplicate code with {@link ChannelTaskConflict}, but that is up for future removal.
 * @author wjschakel
 */
public class ChannelTaskIntersection implements ChannelTask
{

    /** Look-ahead distance. */
    public static final ParameterTypeLength LOOKAHEAD = ParameterTypes.LOOKAHEAD;

    /** Maximum ego task demand. */
    public static final ParameterTypeDouble TD_EGO =
            new ParameterTypeDouble("TD_EGO", "Maximum task demand due to ego distance to intersection.",
                    (1.0 - 0.175) * 0.42 / (0.42 + 0.11), DualBound.UNITINTERVAL);

    /** Ego decay parameter. */
    public static final ParameterTypeLength XEGO = new ParameterTypeLength("x_ego",
            "Exponential decay of conflict task by ego distance.", Length.instantiateSI(32.5), NumericConstraint.POSITIVEZERO);

    /** Maximum other task demand. */
    public static final ParameterTypeDouble TD_OTH =
            new ParameterTypeDouble("TD_OTH", "Maximum task demand due to other vehicle time to conflict.",
                    (1.0 - 0.175) * 0.11 / (0.42 + 0.11), DualBound.UNITINTERVAL);

    /** Conflicting decay parameter. */
    public static final ParameterTypeDuration HCONF = ChannelTaskConflict.HCONF;
    // new ParameterTypeDuration("h_conf", "Exponential decay of conflict task by conflicting approaching time.",
    // Duration.instantiateSI(2.49), NumericConstraint.POSITIVEZERO);

    /** Comparator for underlying objects. */
    // TODO: remove this and its use once UnderlyingDistance implements Comparable
    private static final Comparator<UnderlyingDistance<Conflict>> COMPARATOR = new Comparator<>()
    {
        /** {@inheritDoc} */
        @Override
        public int compare(final UnderlyingDistance<Conflict> o1, final UnderlyingDistance<Conflict> o2)
        {
            int out = o1.getDistance().compareTo(o2.getDistance());
            if (out != 0)
            {
                return out;
            }
            return o1.getObject().getFullId().compareTo(o2.getObject().getFullId());
        }
    };

    /**
     * Standard supplier that supplies a task per grouped set of conflicts based on common upstream nodes. This also adds a
     * scanning task demand to each of these channels.
     */
    public static final Function<LanePerception, Set<ChannelTask>> SUPPLIER = (perception) ->
    {
        Set<ChannelTask> tasksOut = new LinkedHashSet<>();
        ChannelMental channelMental = (perception.getMental() instanceof ChannelMental m) ? m : null;
        Set<SortedSet<UnderlyingDistance<Conflict>>> groups = findConflictGroups(perception);
        IntersectionTaskGroup intersectionTaskGroup = new IntersectionTaskGroup();
        if (!groups.isEmpty())
        {
            // groups are inherently ordered as perception returns conflicts from close to far
            UnderlyingDistance<Conflict> first = groups.iterator().next().first();
            tasksOut.add(new ChannelTaskIntersection(perception.getGtu(), first, new TreeSet<>(), intersectionTaskGroup));
            for (SortedSet<UnderlyingDistance<Conflict>> group : groups)
            {
                splitCarFollowing(tasksOut, group, channelMental);
                if (!group.isEmpty())
                {
                    tasksOut.add(new ChannelTaskIntersection(perception.getGtu(), first, group, intersectionTaskGroup));
                    tasksOut.add(new ChannelTaskScan(group.first().getObject()));
                    // make sure the channel (key is first conflict) can be found for all individual conflicts
                    if (channelMental != null)
                    {
                        group.forEach((c) -> channelMental.mapToChannel(c.getObject(), group.first().getObject()));
                    }
                }
            }
        }
        return tasksOut;
    };

    /** GTU. */
    private final LaneBasedGtu gtu;

    /** First conflict on intersection. */
    private final UnderlyingDistance<Conflict> first;

    /** Conflicts in the group. */
    private final SortedSet<UnderlyingDistance<Conflict>> conflicts;

    /** Group of all instantaneous intersection tasks. */
    private final IntersectionTaskGroup intersectionTaskGroup;

    /** Conflicting task demand. */
    private Double conflictingTaskDemand;

    /** Maximum visibility of conflicts in group. */
    private Length maxVisibility;

    /** Task id for FRONT, which is used when no conflict has any weight. */
    private String emptyId;

    /**
     * Constructor.
     * @param gtu GTU
     * @param first first conflict in the intersection
     * @param conflicts conflicts in the group
     * @param intersectionTaskGroup group of all instantaneous intersection tasks
     */
    private ChannelTaskIntersection(final LaneBasedGtu gtu, final UnderlyingDistance<Conflict> first,
            final SortedSet<UnderlyingDistance<Conflict>> conflicts, final IntersectionTaskGroup intersectionTaskGroup)
    {
        this.gtu = gtu;
        this.first = first;
        this.conflicts = conflicts;
        this.intersectionTaskGroup = intersectionTaskGroup;
        intersectionTaskGroup.addTask(this);
    }

    /** {@inheritDoc} */
    @Override
    public String getId()
    {
        if (this.conflicts.isEmpty())
        {
            if (this.emptyId == null)
            {
                this.emptyId = UUID.randomUUID().toString();
            }
            return this.emptyId;
        }
        return this.conflicts.first().getObject().getFullId();
    }

    /** {@inheritDoc} */
    @Override
    public Object getChannel()
    {
        return this.conflicts.isEmpty() ? FRONT : this.conflicts.first().getObject();
    }

    /** {@inheritDoc} */
    @Override
    public double getDemand(final LanePerception perception)
    {
        Length xEgo = Try.assign(() -> this.gtu.getParameters().getParameter(XEGO), "Parameter x_ego not present.");
        double tdEgo = Try.assign(() -> this.gtu.getParameters().getParameter(TD_EGO), "Parameter TD_EGO not present.");
        double egoDistance = this.first.getDistance().si < 0.0 ? 0.0 : this.first.getDistance().si;
        double td = this.intersectionTaskGroup.getWeightedFactor(this) * tdEgo * Math.exp(-egoDistance / xEgo.si)
                + getConflictingTaskDemand();
        return td;
    }

    /**
     * Returns the relevance of this specific channel in the general intersection context.
     * @return relevance of this specific channel in the general intersection context
     */
    private double getWeight()
    {
        double confTD = getConflictingTaskDemand();
        /*
         * This factor is an integral over 1-exp(-visibility/xEgo) from 0 to visibility (scaled down by xEgo because only
         * relative is relevant). This sums the relevance of the visual area.
         */
        double integral = 1.0 - Math.exp(-this.maxVisibility.si / 32.5);
        return this.conflicts.isEmpty() ? 0.0 : confTD * integral;
    }

    /**
     * Returns conflicting task demand.
     * @return conflicting task demand
     */
    private double getConflictingTaskDemand()
    {
        if (this.conflictingTaskDemand == null)
        {
            Duration conflictingTimeToConflict = Duration.POSITIVE_INFINITY;
            this.maxVisibility = Length.ZERO;
            try
            {
                Length x0 = this.gtu.getParameters().getParameter(LOOKAHEAD);
                for (UnderlyingDistance<Conflict> conflict : this.conflicts)
                {
                    if (conflict.getDistance().ge0())
                    {
                        Length visibility;
                        if (this.gtu.getTacticalPlanner().getPerception() instanceof VisibilityLanePerception vis)
                        {
                            visibility = vis.getVisibility(this.gtu, conflict.getObject().getOtherConflict());
                            // if ("ego".equals(this.gtu.getId().toLowerCase()))
                            // {
                            // System.out.println("Visibility upstream of " + conflict.getObject().getConflictType()
                            // + " on link " + conflict.getObject().getOtherConflict().getLane().getLink().getId()
                            // + ": " + visibility);
                            // }
                        }
                        else
                        {
                            visibility = x0; // Length.POSITIVE_INFINITY;
                        }
                        this.maxVisibility = Length.max(this.maxVisibility, visibility);
                        PerceptionCollectable<HeadwayGtu, LaneBasedGtu> conflictingGtus = conflict.getObject()
                                .getOtherConflict().getUpstreamGtus(this.gtu, HeadwayGtuType.WRAP, Length.min(visibility, x0));
                        if (conflictingGtus.isEmpty())
                        {
                            // ghost vehicle at visibility range (but never closer than ego is to conflict) and speed limit
                            Speed speedLimit;
                            try
                            {
                                speedLimit = conflict.getObject().getOtherConflict().getLane().getHighestSpeedLimit();
                            }
                            catch (NetworkException ex)
                            {
                                speedLimit = this.gtu.getDesiredSpeed();
                            }

                            // Length effectiveDistance = Length.max(visibility, conflict.getDistance());
                            conflictingTimeToConflict = Duration.min(conflictingTimeToConflict, visibility.divide(speedLimit));
                            // conflictingTimeToConflict = Duration.POSITIVE_INFINITY;
                        }
                        else
                        {
                            HeadwayGtu conflictingGtu = conflictingGtus.first();
                            conflictingTimeToConflict = Duration.min(conflictingTimeToConflict, conflictingGtu.isParallel()
                                    ? Duration.ZERO : conflictingGtu.getDistance().divide(conflictingGtu.getSpeed()));
                        }
                    }
                }
            }
            catch (ParameterException ex)
            {
                throw new OtsRuntimeException(ex);
            }
            double tdOth = Try.assign(() -> this.gtu.getParameters().getParameter(TD_OTH), "Parameter TD_OTH not present.");
            Duration hConf = Try.assign(() -> this.gtu.getParameters().getParameter(HCONF), "Parameter h_conf not present.");
            this.conflictingTaskDemand = tdOth * Math.exp(-conflictingTimeToConflict.si / hConf.si);
        }

        return this.conflictingTaskDemand;
    }

    /**
     * Returns conflict groups, which are grouped based on overlap in the upstream nodes of the conflicting lanes.
     * @param perception perception
     * @return conflict groups
     */
    private static Set<SortedSet<UnderlyingDistance<Conflict>>> findConflictGroups(final LanePerception perception)
    {
        IntersectionPerception intersection =
                Try.assign(() -> perception.getPerceptionCategory(IntersectionPerception.class), "No intersection perception.");
        Iterator<UnderlyingDistance<Conflict>> conflicts =
                intersection.getConflicts(RelativeLane.CURRENT).underlyingWithDistance();

        // Find groups of conflicts when their upstream nodes are intersecting sets
        Map<SortedSet<UnderlyingDistance<Conflict>>, Set<Node>> groups = new LinkedHashMap<>();
        Length x0 = Try.assign(() -> perception.getGtu().getParameters().getParameter(LOOKAHEAD), "No x0 parameter.");
        while (conflicts.hasNext())
        {
            UnderlyingDistance<Conflict> conflict = conflicts.next();
            Set<Node> nodes = getUpstreamNodes(conflict.getObject().getOtherConflict(), x0);
            // find overlap
            Entry<SortedSet<UnderlyingDistance<Conflict>>, Set<Node>> group = null;
            Iterator<Entry<SortedSet<UnderlyingDistance<Conflict>>, Set<Node>>> groupIterator = groups.entrySet().iterator();
            while (groupIterator.hasNext())
            {
                Entry<SortedSet<UnderlyingDistance<Conflict>>, Set<Node>> entry = groupIterator.next();
                if (entry.getValue().stream().anyMatch(nodes::contains))
                {
                    // overlap with this entry
                    if (group == null)
                    {
                        entry.getKey().add(conflict);
                        entry.getValue().addAll(nodes);
                        group = entry;
                        // keep looping to also merge other groups if they overlap with the upstream nodes of this conflict
                    }
                    else
                    {
                        // the nodes overlap with multiple groups that did so far not yet overlap, merge the other group too
                        group.getKey().addAll(entry.getKey());
                        group.getValue().addAll(entry.getValue());
                        groupIterator.remove();
                    }
                }
            }
            if (group == null)
            {
                // no overlap found, make new group
                // TODO: remove COMPARATOR argument once UnderlyingDistance implements Comparable
                SortedSet<UnderlyingDistance<Conflict>> key = new TreeSet<>(COMPARATOR);
                key.add(conflict);
                groups.put(key, nodes);
            }
        }
        return groups.keySet();
    }

    /**
     * Finds all nodes within a given distance upstream of a conflict, stopping at any diverge, branging at merges.
     * @param conflict conflict.
     * @param x0 distance to loop upstream.
     * @return set of all upstream nodes within distance.
     */
    private static Set<Node> getUpstreamNodes(final Conflict conflict, final Length x0)
    {
        Set<Node> nodes = new LinkedHashSet<>();
        Link link = conflict.getLane().getLink();
        Length distance = link.getLength().times(conflict.getLane().fraction(conflict.getLongitudinalPosition()) - 1.0);
        appendUpstreamNodes(link, distance, x0, nodes);
        return nodes;
    }

    /**
     * Append upstream nodes, branging upstream at merges, stopping at any diverge.
     * @param link next link to move along.
     * @param distance distance between end of link and conflict, upstream of conflict.
     * @param x0 search distance.
     * @param nodes collected nodes.
     */
    private static void appendUpstreamNodes(final Link link, final Length distance, final Length x0, final Set<Node> nodes)
    {
        Length nextDistance = distance.plus(link.getLength());
        if (nextDistance.le(x0))
        {
            Node start = link.getStartNode();
            ImmutableSet<Link> links = start.getLinks();
            Set<Link> upstreamLinks = new LinkedHashSet<>();
            for (Link next : links)
            {
                if (!next.equals(link))
                {
                    if (next.getStartNode().equals(start))
                    {
                        // diverge
                        nodes.add(start);
                        return;
                    }
                    upstreamLinks.add(next);
                }
            }
            nodes.add(start);
            for (Link upstreamLink : upstreamLinks)
            {
                appendUpstreamNodes(upstreamLink, nextDistance, x0, nodes);
            }
        }
    }

    /**
     * Apply car-following task on each split in the group, and remove it from the group.
     * @param tasks tasks to add any split related task to
     * @param group group of conflicts
     * @param channelMental mental module, can be {@code null}
     */
    private static void splitCarFollowing(final Set<ChannelTask> tasks, final SortedSet<UnderlyingDistance<Conflict>> group,
            final ChannelMental channelMental)
    {
        Iterator<UnderlyingDistance<Conflict>> iterator = group.iterator();
        while (iterator.hasNext())
        {
            UnderlyingDistance<Conflict> conflict = iterator.next();
            if (conflict.getObject().getConflictType().isSplit())
            {
                iterator.remove();
                tasks.add(new ChannelTaskCarFollowing((p) ->
                {
                    // this provides the first leader on the other split conflict with distance towards perceiving GTU
                    Conflict otherconflict = conflict.getObject().getOtherConflict();
                    PerceptionCollectable<HeadwayGtu, LaneBasedGtu> conflictingGtus =
                            otherconflict.getDownstreamGtus(p.getGtu(), HeadwayGtuType.WRAP, otherconflict.getLength());
                    if (conflictingGtus.isEmpty())
                    {
                        return null;
                    }
                    UnderlyingDistance<LaneBasedGtu> leader = conflictingGtus.underlyingWithDistance().next();
                    return new UnderlyingDistance<LaneBasedGtu>(leader.getObject(),
                            conflict.getDistance().plus(leader.getDistance()));
                }));
                // make sure the channel (key is front) can be found for the split conflict
                if (channelMental != null)
                {
                    channelMental.mapToChannel(conflict.getObject(), FRONT);
                }
            }
        }
    }

    /**
     * Group of intersection tasks.
     */
    private static class IntersectionTaskGroup
    {

        /** Set of currently relevant intersection tasks. */
        private final Map<ChannelTaskIntersection, Double> weights = new LinkedHashMap<>();

        /** Total weight. */
        private double totalWeight = 0.0;

        /**
         * Add task.
         * @param task
         */
        public void addTask(final ChannelTaskIntersection task)
        {
            double weight = task.getWeight();
            this.weights.put(task, weight);
            this.totalWeight += weight;
        }

        /**
         * Get weighted factor of task demand for this specific task (for a specific channel).
         * @param channelTaskIntersection task
         * @return weighted factor of task demand for this specific task (for a specific channel)
         */
        public double getWeightedFactor(final ChannelTaskIntersection channelTaskIntersection)
        {
            if (channelTaskIntersection.conflicts.isEmpty())
            {
                return this.totalWeight == 0.0 ? 1.0 : 0.0;
            }
            return this.totalWeight == 0.0 ? 0.0 : (this.weights.get(channelTaskIntersection) / this.totalWeight);
        }

    }

}
