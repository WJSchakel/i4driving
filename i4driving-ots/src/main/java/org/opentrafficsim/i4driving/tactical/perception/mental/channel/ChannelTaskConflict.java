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
import java.util.function.Function;

import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djutils.exceptions.Try;
import org.djutils.immutablecollections.ImmutableSet;
import org.opentrafficsim.base.parameters.ParameterTypeDuration;
import org.opentrafficsim.base.parameters.ParameterTypeLength;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.core.gtu.perception.EgoPerception;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.i4driving.Stateless;
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
 * Task demand for a group of conflicts pertaining to the same conflicting road. This is defined as
 * {@code exp(-max(T_ego, min(T_conf))/h)} where {@code T_ego} is the ego time until the first conflict, {@code T_conf} is the
 * time until the respective conflict of a conflicting vehicle and {@code h} is the car-following task parameter that scales it.
 * @author wjschakel
 */
@Stateless
public final class ChannelTaskConflict implements ChannelTask
{
    /** Look-ahead distance. */
    public static final ParameterTypeLength LOOKAHEAD = ParameterTypes.LOOKAHEAD;

    /** Ego decay parameter. */
    public static final ParameterTypeDuration HEGO =
            new ParameterTypeDuration("h_ego", "Exponential decay of conflict task by ego approaching time.",
                    Duration.instantiateSI(4.46), NumericConstraint.POSITIVEZERO);

    /** Conflicting decay parameter. */
    public static final ParameterTypeDuration HCONF =
            new ParameterTypeDuration("h_conf", "Exponential decay of conflict task by conflicting approaching time.",
                    Duration.instantiateSI(2.49), NumericConstraint.POSITIVEZERO);

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
        Set<ChannelTask> tasks = new LinkedHashSet<>();
        ChannelMental channelMental = (perception.getMental() instanceof ChannelMental m) ? m : null;
        for (SortedSet<UnderlyingDistance<Conflict>> group : findConflictGroups(perception))
        {
            splitCarFollowing(tasks, group, channelMental);
            if (!group.isEmpty())
            {
                tasks.add(new ChannelTaskConflict(group));
                tasks.add(new ChannelTaskScan(group.first().getObject()));
                // make sure the channel (key is first conflict) can be found for all individual conflicts
                if (channelMental != null)
                {
                    group.forEach((c) -> channelMental.mapToChannel(c.getObject(), group.first().getObject()));
                }
            }
        }
        return tasks;
    };

    /** Conflicts in the group. */
    private final SortedSet<UnderlyingDistance<Conflict>> conflicts;

    /**
     * Constructor.
     * @param conflicts conflicts in the group.
     */
    private ChannelTaskConflict(final SortedSet<UnderlyingDistance<Conflict>> conflicts)
    {
        this.conflicts = conflicts;
    }

    /** {@inheritDoc} */
    @Override
    public String getId()
    {
        return this.conflicts.first().getObject().getFullId();
    }

    /** {@inheritDoc} */
    @Override
    public Object getChannel()
    {
        return this.conflicts.first().getObject();
    }

    /** {@inheritDoc} */
    @Override
    public double getDemand(final LanePerception perception)
    {
        // In the following, 'headway' means time until static conflict is reached, i.e. approaching time.

        // Get minimum headway of first vehicle on each conflict in the group
        Duration conflictHeadway = Duration.POSITIVE_INFINITY;
        LaneBasedGtu gtu = Try.assign(() -> perception.getGtu(), "Gtu not initialized.");
        Length x0 = Try.assign(() -> perception.getGtu().getParameters().getParameter(LOOKAHEAD), "No x0 parameter.");
        for (UnderlyingDistance<Conflict> conflict : this.conflicts)
        {
            PerceptionCollectable<HeadwayGtu, LaneBasedGtu> conflictingGtus =
                    conflict.getObject().getOtherConflict().getUpstreamGtus(gtu, HeadwayGtuType.WRAP, x0);
            if (!conflictingGtus.isEmpty())
            {
                HeadwayGtu conflictingGtu = conflictingGtus.first();
                conflictHeadway = Duration.min(conflictHeadway, conflictingGtu.isParallel() ? Duration.ZERO
                        : conflictingGtu.getDistance().divide(conflictingGtu.getSpeed()));
            }
        }

        // Get own approaching time
        EgoPerception<?, ?> ego =
                Try.assign(() -> perception.getPerceptionCategory(EgoPerception.class), "EgoPerception not present.");
        Duration egoHeadway = this.conflicts.first().getDistance().divide(ego.getSpeed());

        // Find least critical
        Duration hEgo =
                Try.assign(() -> perception.getGtu().getParameters().getParameter(HEGO), "Parameter h_ego not present.");
        Duration hConf =
                Try.assign(() -> perception.getGtu().getParameters().getParameter(HCONF), "Parameter h_conf not present.");
        return Math.min(0.999, Math.exp(-Math.min(egoHeadway.si / hEgo.si, conflictHeadway.si / hConf.si)));
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

}
