#ifndef SPATIAL_PROPERTIES_ICE
#define SPATIAL_PROPERTIES_ICE

#include <cast/slice/CDL.ice>
#include <SpatialData.ice>

/**
 * Defines data structures representing properties and semantic 
 * category of places and other spatial units.
 *
 * @author Andrzej Pronobis
 */
module SpatialProperties
{

  // -----------------------------------------------------------------
  // Property values
  // -----------------------------------------------------------------

  /** Generic, abstract class for all property values. */
  class PropertyValue
  {
  };

  /** Class representing binary values. */
  class BinaryValue extends PropertyValue
  {
    bool value;
  };

  /** Class representing integer values. */
  class IntegerValue extends PropertyValue
  {
    long value;
  };

  /** Class representing floating point values. */
  class FloatValue extends PropertyValue
  {
    double value;
  };

  /** Class representing string values. */
  class StringValue extends PropertyValue
  {
    string value;
  };
  

  // -----------------------------------------------------------------
  // Probability distributions
  // -----------------------------------------------------------------

  /** Assignment between a value and its probability. */
  struct ValueProbabilityPair
  {
    PropertyValue value;
    double probability;
  };

  /** Sequence of value-probability pairs. */
  sequence <ValueProbabilityPair> ValueProbabilityPairs;

  /** Generic, abstract class for all probability distributions. */
  class ProbabilityDistribution
  {
  };

  /** Class representing a sparse, discrete probability distribution. */
  class DiscreteProbabilityDistribution extends ProbabilityDistribution
  {
    ValueProbabilityPairs data;
  };


  // -----------------------------------------------------------------
  // Place properties
  // -----------------------------------------------------------------

  /** 
   * Generic, abstract class representing the distribution of values 
   * of a single property of a single place.
   */
  class PlaceProperty
  {
    /** ID of the place that has this property. */
    long placeId;
    
    /** Probability distribution of values of this property. */
    ProbabilityDistribution distribution;

    /** Maximum a posteriori estimate of the property value. */
    PropertyValue mapValue;

    /** True if the MAP estimate is considered as reliable. */
    bool mapValueReliable;
  };

  /** Represents the gateway property of a place. */
  class GatewayPlaceProperty extends PlaceProperty
  {
  };
  const bool IsGateway = true;
  const bool IsNotGateway = false;

  /** Represents the object-based property of a place for a single object. */
  class ObjectPlaceProperty extends PlaceProperty
  {
    string category;
  };
  const bool ObjectPresent = true;
  const bool ObjectNotPresent = false;

  /** Represents the room appearance class property for a place. */
  class RoomAppearancePlaceProperty extends PlaceProperty
  {
  };

  /** Represents the room geometry class property for a place. */
  class RoomGeometryPlaceProperty extends PlaceProperty
  {
  };

  /** Represents the room shape property for a place. */
  class RoomShapePlaceProperty extends PlaceProperty
  {
  };

  /** Represents the room size property for a place. */
  class RoomSizePlaceProperty extends PlaceProperty
  {
  };

  /** Represents the amount of free space (not associated with
   an existing node) that is closest to this Placeholder*/
  class AssociatedSpacePlaceholderProperty extends PlaceProperty
  {
    float value;
  };

  /** Represents the amount of unknown space boundary 
    that is closest to this Placeholder*/
  class AssociatedBorderPlaceholderProperty extends PlaceProperty
  {
    float value;
  };

  /** 
   * Class representing the semantic room category as observed from
   * within a place located in some room in terms of a distribution
   * over possible room categories.
   */
  class PlaceRoomCategory extends PlaceProperty
  {
  };

  /** 
   * Generic, abstract class representing the distribution of values 
   * of a single property of a single path.
   */
  class PathProperty
  {
    /** IDs of the places defining the path that has this property. */
    long place1Id;
    long place2Id;
    
    /** Probability distribution of values of this property. */
    ProbabilityDistribution distribution;

    /** Maximum a posteriori estimate of the property value. */
    PropertyValue mapValue;

    /** True if the MAP estimate is considered as reliable. */
    bool mapValueReliable;
  };

  /**
   * Represents the navigable connectivity of two Places
   * (one-way).
   * Transition success probability is encoded in the
   * probability of transition cost not being infinity.
   */
  class ConnectivityPathProperty extends PathProperty
  {
  };

  // -----------------------------------------------------------------
  // Room properties
  // -----------------------------------------------------------------

  /** 
   * Generic, abstract class representing the distribution of values 
   * of a single property of a single room.
   */
  class RoomProperty
  {
    /** ID of the room that has this property. */
    long roomId;
    
    /** Probability distribution of values of this property. */
    ProbabilityDistribution distribution;

    /** Maximum a posteriori estimate of the property value. */
    PropertyValue mapValue;

    /** True if the MAP estimate is considered as reliable. */
    bool mapValueReliable;
  };

  /** Represents the room appearance class property for a room. */
  class RoomAppearanceRoomProperty extends RoomProperty
  {
  };

  /** Represents the room geometry class property for a room. */
  class RoomGeometryRoomProperty extends RoomProperty
  {
  };

  /** Represents the room shape property for a room. */
  class RoomShapeRoomProperty extends RoomProperty
  {
  };

  /** Represents the room size property for a room. */
  class RoomSizeRoomProperty extends RoomProperty
  {
  };

  /** 
   * Class representing the semantic category of a room
   * in terms of a distribution over possible room categories.
   */
  class RoomCategory extends RoomProperty
  {
  };

};

#endif // SPATIAL_PROPERTIES_ICE
