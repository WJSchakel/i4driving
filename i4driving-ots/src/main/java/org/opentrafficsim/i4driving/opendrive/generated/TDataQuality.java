
package org.opentrafficsim.i4driving.opendrive.generated;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlType;


/**
 * Raw data or data from external sources that is integrated in OpenDRIVE may be of varying quality. It is possible to describe quality and accuracy of external data in OpenDRIVE.
 * The description of the data quality is represented by <dataQuality> elements. They may be stored at any position in OpenDRIVE.
 * Measurement data derived from external sources like GPS that is integrated in OpenDRIVE may be inaccurate. The error range, given in [m], may be listed in the application.
 * 
 * <p>Java class for t_dataQuality complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="t_dataQuality"&gt;
 *   &lt;complexContent&gt;
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType"&gt;
 *       &lt;sequence&gt;
 *         &lt;element name="error" type="{http://code.asam.net/simulation/standard/opendrive_schema}t_dataQuality_Error" minOccurs="0"/&gt;
 *         &lt;element name="rawData" type="{http://code.asam.net/simulation/standard/opendrive_schema}t_dataQuality_RawData" minOccurs="0"/&gt;
 *       &lt;/sequence&gt;
 *     &lt;/restriction&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "t_dataQuality", propOrder = {
    "error",
    "rawData"
})
@SuppressWarnings("all") public class TDataQuality {

    protected TDataQualityError error;
    protected TDataQualityRawData rawData;

    /**
     * Gets the value of the error property.
     * 
     * @return
     *     possible object is
     *     {@link TDataQualityError }
     *     
     */
    public TDataQualityError getError() {
        return error;
    }

    /**
     * Sets the value of the error property.
     * 
     * @param value
     *     allowed object is
     *     {@link TDataQualityError }
     *     
     */
    public void setError(TDataQualityError value) {
        this.error = value;
    }

    /**
     * Gets the value of the rawData property.
     * 
     * @return
     *     possible object is
     *     {@link TDataQualityRawData }
     *     
     */
    public TDataQualityRawData getRawData() {
        return rawData;
    }

    /**
     * Sets the value of the rawData property.
     * 
     * @param value
     *     allowed object is
     *     {@link TDataQualityRawData }
     *     
     */
    public void setRawData(TDataQualityRawData value) {
        this.rawData = value;
    }

}