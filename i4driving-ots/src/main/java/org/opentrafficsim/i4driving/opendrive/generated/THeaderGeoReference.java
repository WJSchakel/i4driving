
package org.opentrafficsim.i4driving.opendrive.generated;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import javax.xml.bind.JAXBElement;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElementRef;
import javax.xml.bind.annotation.XmlElementRefs;
import javax.xml.bind.annotation.XmlMixed;
import javax.xml.bind.annotation.XmlType;


/**
 * Spatial reference systems are standardized by the European Petroleum Survey Group Geodesy (EPSG) and are defined by parameters describing the geodetic datum. A geodetic datum is a coordinate reference system for a collection of positions that are relative to an ellipsoid model of the earth. 
 * A geodetic datum is described by a projection string according to PROJ, that is, a format for the exchange of data between two coordinate systems. This data shall be marked as CDATA, because it may contain characters that interfere with the XML syntax of an element’s attribute.
 * In OpenDRIVE, the information about the geographic reference of an OpenDRIVE dataset is represented by the <geoReference> element within the <header> element.
 * 
 * <p>Java class for t_header_GeoReference complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="t_header_GeoReference"&gt;
 *   &lt;complexContent&gt;
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType"&gt;
 *       &lt;sequence&gt;
 *         &lt;group ref="{http://code.asam.net/simulation/standard/opendrive_schema}g_additionalData" maxOccurs="unbounded" minOccurs="0"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/restriction&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "t_header_GeoReference", propOrder = {
    "content"
})
@SuppressWarnings("all") public class THeaderGeoReference {

    @XmlElementRefs({
        @XmlElementRef(name = "include", namespace = "http://code.asam.net/simulation/standard/opendrive_schema", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "userData", namespace = "http://code.asam.net/simulation/standard/opendrive_schema", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "dataQuality", namespace = "http://code.asam.net/simulation/standard/opendrive_schema", type = JAXBElement.class, required = false)
    })
    @XmlMixed
    protected List<Serializable> content;

    /**
     * Spatial reference systems are standardized by the European Petroleum Survey Group Geodesy (EPSG) and are defined by parameters describing the geodetic datum. A geodetic datum is a coordinate reference system for a collection of positions that are relative to an ellipsoid model of the earth. 
     * A geodetic datum is described by a projection string according to PROJ, that is, a format for the exchange of data between two coordinate systems. This data shall be marked as CDATA, because it may contain characters that interfere with the XML syntax of an element’s attribute.
     * In OpenDRIVE, the information about the geographic reference of an OpenDRIVE dataset is represented by the <geoReference> element within the <header> element.Gets the value of the content property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the content property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getContent().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link String }
     * {@link JAXBElement }{@code <}{@link TDataQuality }{@code >}
     * {@link JAXBElement }{@code <}{@link TInclude }{@code >}
     * {@link JAXBElement }{@code <}{@link TUserData }{@code >}
     * 
     * 
     */
    public List<Serializable> getContent() {
        if (content == null) {
            content = new ArrayList<Serializable>();
        }
        return this.content;
    }

}