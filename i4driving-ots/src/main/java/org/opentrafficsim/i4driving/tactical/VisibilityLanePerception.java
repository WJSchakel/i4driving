package org.opentrafficsim.i4driving.tactical;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import org.djunits.value.vdouble.scalar.Length;
import org.djutils.draw.line.PolyLine2d;
import org.djutils.draw.line.Polygon2d;
import org.djutils.draw.point.Point2d;
import org.djutils.exceptions.Try;
import org.djutils.logger.CategoryLogger;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.CategoricalLanePerception;
import org.opentrafficsim.road.gtu.lane.perception.mental.Mental;
import org.opentrafficsim.road.network.lane.object.LaneBasedObject;

/**
 * Lane perception that can provide visibility upstream of objects.
 * @author wjschakel
 */
public class VisibilityLanePerception extends CategoricalLanePerception
{

    /** */
    private static final long serialVersionUID = 1L;

    /** Visibility. */
    private final Visibility visibility;

    /**
     * Create a new LanePerception module with mental module.
     * @param gtu LaneBasedGtu; GTU
     * @param mental Mental; mental module
     */
    public VisibilityLanePerception(final LaneBasedGtu gtu, final Mental mental, final Visibility visibility)
    {
        super(gtu, mental);
        this.visibility = visibility;
    }

    /**
     * Return the visibility range upstream of the given object.
     * @param perceivingGtu perceiving GTU
     * @param object object
     * @return visibility range upstream of the given object
     * @throws ParameterException when look-ahead parameter is not given
     */
    public Length getVisibility(final LaneBasedGtu perceivingGtu, final LaneBasedObject object) throws ParameterException
    {
        return this.visibility == null ? perceivingGtu.getParameters().getParameter(LOOKAHEAD)
                : this.visibility.getVisibility(perceivingGtu, object);
    }

    /**
     * This class stores visibility anchor points.
     */
    public static class Visibility
    {

        /** Anchors. */
        private final Map<Link, Map<Link, Set<Point2d>>> anchors = new LinkedHashMap<>();

        /**
         * Add anchor. Visibility from the from-link to the to-link is along a ray from any GTU on the from-link over the anchor
         * and projected to the to-link. Visibility is downstream from this point to a relevant object, either on this link or
         * on a downstream link.
         * @param from from-link
         * @param to to-link
         * @param anchor anchor point
         */
        public void addAnchor(final Link from, final Link to, final Point2d anchor)
        {
            this.anchors.computeIfAbsent(from, (l) -> new LinkedHashMap<>()).computeIfAbsent(to, (l) -> new LinkedHashSet<>())
                    .add(anchor);
        }

        /**
         * Return the visibility range upstream of the given object.
         * @param perceivingGtu perceiving GTU
         * @param object object
         * @return visibility range upstream of the given object
         * @throws ParameterException when look-ahead parameter is not given
         */
        public Length getVisibility(final LaneBasedGtu perceivingGtu, final LaneBasedObject object) throws ParameterException
        {
            Length x0 = perceivingGtu.getParameters().getParameter(LOOKAHEAD);
            Link from = Try.assign(() -> perceivingGtu.getReferencePosition().lane().getLink(),
                    "GTU does not have a valid reference position.");
            if (!this.anchors.containsKey(from))
            {
                return x0;
            }
            Map<Link, Set<Point2d>> toMap = this.anchors.get(from);
            Link to = object.getLane().getLink();

            double fracPosition = object.getLongitudinalPosition().si / object.getLane().getLength().si;
            Length cumul = to.getLength().times(fracPosition - 1.0);

            while (to != null && cumul.lt(x0))
            {
                if (toMap.containsKey(to))
                {
                    Length minimumVisibility = null;
                    for (Point2d anchor : toMap.get(to))
                    {
                        Optional<Length> linkVisibility = computeVisibility(perceivingGtu, to, anchor, cumul);
                        if (linkVisibility.isPresent() && linkVisibility.get().ge0())
                        {
                            minimumVisibility = minimumVisibility == null ? linkVisibility.get()
                                    : Length.min(minimumVisibility, linkVisibility.get());
                        }
                    }
                    if (minimumVisibility != null)
                    {
                        return Length.min(x0, minimumVisibility);
                    }
                }

                Link candidate = null;
                for (Link next : to.getStartNode().getLinks())
                {
                    if (!next.equals(to) && next.getEndNode().equals(to.getStartNode()))
                    {
                        if (candidate != null)
                        {
                            to = null; // multiple upstream links
                            break;
                        }
                        cumul = cumul.plus(to.getLength());
                        candidate = next;
                    }
                }
                to = candidate;
            }
            return x0;
        }

        /**
         * Returns the visibility by projecting the ray from the perceiving GTU along the anchor, on to the design line of the
         * to-link.
         * @param perceivingGtu perceiving GTU
         * @param to to-link
         * @param anchor anchor point for visibility
         * @param cumul cumulative upstream distance of intermediate links between relevant object and the end of to-link
         * @return visibility if the view line intersects the to-link
         */
        public Optional<Length> computeVisibility(final LaneBasedGtu perceivingGtu, final Link to, final Point2d anchor,
                final Length cumul)
        {
            PolyLine2d line = to.getDesignLine().getLine2d();
            Point2d pos = perceivingGtu.getLocation();
            for (int i = line.size() - 1; i > 0; i--)
            {
                Point2d intersect = Point2d.intersectionOfLines(pos.x, pos.y, anchor.x, anchor.y, true, false, line.getX(i - 1),
                        line.getY(i - 1), line.getX(i), line.getY(i), true, true);
                if (intersect != null)
                {
                    double fraction = line.projectOrthogonalFractionalExtended(intersect);
                    return Optional.of(cumul.plus(to.getDesignLine().getLength().times(1.0 - fraction)));
                }
            }
            return Optional.empty();
        }

        /**
         * Adds anchor for any combination of links for which the point lies within the polygon created by both links. This is
         * tested for two polygons where one of the links is reversed or not. A polygon is also created for each link
         * individually to test whether the anchor obstructs view within a link.
         * @param network network
         * @param point anchor point
         */
        public void addAnchor(final Network network, final Point2d anchor)
        {
            List<Link> links = new ArrayList<>();
            links.addAll(network.getLinkMap().values().toCollection());
            for (int i = 0; i < links.size(); i++)
            {
                Link link1 = links.get(i);
                for (int j = i; j < links.size(); j++)
                {
                    Set<Polygon2d> polygons;
                    Link link2 = links.get(j);
                    if (i == j)
                    {
                        polygons = Set.of(new Polygon2d(link1.getDesignLine().getLine2d().getPointList()));
                    }
                    else if (link1.getStartNode().equals(link2.getStartNode()))
                    {
                        List<Point2d> list = new ArrayList<>();
                        list.addAll(link1.getDesignLine().getLine2d().getPointList());
                        list.addAll(link2.getDesignLine().getLine2d().reverse().getPointList());
                        list.remove(0);
                        polygons = Set.of(new Polygon2d(list));
                    }
                    else if (link1.getStartNode().equals(link2.getEndNode()))
                    {
                        List<Point2d> list = new ArrayList<>();
                        list.addAll(link1.getDesignLine().getLine2d().getPointList());
                        list.addAll(link2.getDesignLine().getLine2d().getPointList());
                        list.remove(0);
                        polygons = Set.of(new Polygon2d(list));
                    }
                    else if (link1.getEndNode().equals(link2.getStartNode()))
                    {
                        List<Point2d> list = new ArrayList<>();
                        list.addAll(link1.getDesignLine().getLine2d().reverse().getPointList());
                        list.addAll(link2.getDesignLine().getLine2d().reverse().getPointList());
                        list.remove(0);
                        polygons = Set.of(new Polygon2d(list));
                    }
                    else if (link1.getEndNode().equals(link2.getEndNode()))
                    {
                        List<Point2d> list = new ArrayList<>();
                        list.addAll(link1.getDesignLine().getLine2d().reverse().getPointList());
                        list.addAll(link2.getDesignLine().getLine2d().getPointList());
                        list.remove(0);
                        polygons = Set.of(new Polygon2d(list));
                    }
                    else
                    {
                        List<Point2d> list1 = new ArrayList<>();
                        List<Point2d> list2 = new ArrayList<>();
                        list1.addAll(link1.getDesignLine().getLine2d().getPointList());
                        list2.addAll(link1.getDesignLine().getLine2d().getPointList());
                        list1.addAll(link2.getDesignLine().getLine2d().getPointList());
                        list2.addAll(link2.getDesignLine().getLine2d().reverse().getPointList());
                        polygons = Set.of(new Polygon2d(list1), new Polygon2d(list2));
                    }
                    for (Polygon2d polygon : polygons)
                    {
                        if (polygon.contains(anchor))
                        {
                            addAnchor(link1, link2, anchor);
                            addAnchor(link2, link1, anchor);
                            CategoryLogger.always().trace("Adding anchor {} between links {} and {}.", anchor, link1.getId(),
                                    link2.getId());
                            break;
                        }
                    }
                }
            }
        }
    }

}
