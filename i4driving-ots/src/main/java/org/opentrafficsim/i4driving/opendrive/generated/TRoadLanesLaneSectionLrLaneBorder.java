
package org.opentrafficsim.i4driving.opendrive.generated;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * Lane borders are another method to describe the width of lanes. Instead of defining the width directly, lane borders describe the outer limits of a lane, independent of the parameters of their inner borders. In this case, inner lanes are defined as lanes which have the same sign for their ID as the lane currently defined, but with a smaller absolute value for their ID.
 * Especially when road data is derived from automatic measurements, this type of definition is easier than specifying the lane width because it avoids creating many lane sections.
 * Lane width and lane border elements are mutually exclusive within the same lane group. If both width and lane border elements are present for a lane section in the OpenDRIVE file, the application shall use the information from the <width> elements.
 * In OpenDRIVE, lane borders are represented by the <border> element within the <lane> element.
 * 
 * <p>Java class for t_road_lanes_laneSection_lr_lane_border complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="t_road_lanes_laneSection_lr_lane_border"&gt;
 *   &lt;complexContent&gt;
 *     &lt;extension base="{http://code.asam.net/simulation/standard/opendrive_schema}_OpenDriveElement"&gt;
 *       &lt;sequence&gt;
 *       &lt;/sequence&gt;
 *       &lt;attribute name="sOffset" use="required" type="{http://code.asam.net/simulation/standard/opendrive_schema}t_grEqZero" /&gt;
 *       &lt;attribute name="a" use="required" type="{http://www.w3.org/2001/XMLSchema}double" /&gt;
 *       &lt;attribute name="b" use="required" type="{http://www.w3.org/2001/XMLSchema}double" /&gt;
 *       &lt;attribute name="c" use="required" type="{http://www.w3.org/2001/XMLSchema}double" /&gt;
 *       &lt;attribute name="d" use="required" type="{http://www.w3.org/2001/XMLSchema}double" /&gt;
 *     &lt;/extension&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "t_road_lanes_laneSection_lr_lane_border")
@SuppressWarnings("all") public class TRoadLanesLaneSectionLrLaneBorder
    extends OpenDriveElement
{

    @XmlAttribute(name = "sOffset", required = true)
    protected double sOffset;
    @XmlAttribute(name = "a", required = true)
    protected double a;
    @XmlAttribute(name = "b", required = true)
    protected double b;
    @XmlAttribute(name = "c", required = true)
    protected double c;
    @XmlAttribute(name = "d", required = true)
    protected double d;

    /**
     * Gets the value of the sOffset property.
     * 
     */
    public double getSOffset() {
        return sOffset;
    }

    /**
     * Sets the value of the sOffset property.
     * 
     */
    public void setSOffset(double value) {
        this.sOffset = value;
    }

    /**
     * Gets the value of the a property.
     * 
     */
    public double getA() {
        return a;
    }

    /**
     * Sets the value of the a property.
     * 
     */
    public void setA(double value) {
        this.a = value;
    }

    /**
     * Gets the value of the b property.
     * 
     */
    public double getB() {
        return b;
    }

    /**
     * Sets the value of the b property.
     * 
     */
    public void setB(double value) {
        this.b = value;
    }

    /**
     * Gets the value of the c property.
     * 
     */
    public double getC() {
        return c;
    }

    /**
     * Sets the value of the c property.
     * 
     */
    public void setC(double value) {
        this.c = value;
    }

    /**
     * Gets the value of the d property.
     * 
     */
    public double getD() {
        return d;
    }

    /**
     * Sets the value of the d property.
     * 
     */
    public void setD(double value) {
        this.d = value;
    }

}
