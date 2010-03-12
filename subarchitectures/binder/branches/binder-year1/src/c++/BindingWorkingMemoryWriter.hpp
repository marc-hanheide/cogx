#ifndef BINDING_WORKING_MEMORY_WRITER_HPP
#define BINDING_WORKING_MEMORY_WRITER_HPP

#include <cast/architecture/ManagedComponent.hpp>
#include <autogen/BinderEssentials.hpp>

namespace binder {

  /**
   * Abstract class for structuring and inserting proxies into the binder
   * working memory
   *
   * Nick: Return values for the add* methods have been removed to
   * reduce potential ambiguity.
   *
   * @author Pierre Lison, Nick Hawes
   * @version 02/09/2009
   */
  class BindingWorkingMemoryWriter :
    public virtual cast::ManagedComponent {

  public:
      void configure(const std::map<std::string, std::string>& _config);

  private:
    /**
     * Name of the binder subarchitecture
     */
    std::string m_bindingSA;

    /**
     * ID in current SA where origin map is stored.
     */
    std::string m_originMapID;

    /**
     * Stores a mapping from the source to the proxy which is created from it.
     *
     * @param _proxy
     * @throws AlreadyExistsOnWMException
     */
    void storeOriginInfo(autogen::core::ProxyPtr _proxy);

    /**
     * Removes source id mapping from WM map.
     *
     * @param _proxy
     * @throws DoesNotExistOnWMException
     * @throws ConsistencyException
     * @throws PermissionException
     */
    void removeOriginInfo(autogen::core::ProxyPtr _proxy);

  protected:


    cast::cdl::WorkingMemoryPointerPtr createWorkingMemoryPointer (const cast::cdl::WorkingMemoryAddress wma,
						   const std::string &  localDataType) {
      return createWorkingMemoryPointer(wma.subarchitecture, wma.id, localDataType);
    }


    cast::cdl::WorkingMemoryPointerPtr createWorkingMemoryPointer (const std::string & subarchId,
         const std::string &  localDataId, const std::string &  localDataType);



    /** Create a new proxy given the ID of the originating subarchitecture,
     * and the probability of the proxy itself
     * (the list of features is defined to be empty)
     *
     * @param subarchId string for the ID of the subarchitecture
     * @param probExists probability value for the proxy
     * @return a new proxy
     */
    autogen::core::ProxyPtr createNewProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
					    float probExists);


    /**
	 * Create a new relation proxy given the ID of the originating subarchitecture,
	 * the probability of the proxy, and the source and target proxies
	 *
	 * @param subarchId string for the ID of the subarchitecture
	 * @param probExists the probability of the proxy
	 * @param sourceProxy the source proxy
	 * @param targetProxy the target proxy
	 * @return the new relation proxy
	 */
    autogen::specialentities::RelationProxyPtr createNewRelationProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
						    float probExists,
						    const autogen::core::FeatureValues source,
						    const autogen::core::FeatureValues target);



	/**
	 * Create a new relation proxy given the ID of the originating subarchitecture,
	 * the probability of the proxy, the list of features for the relation,
	 * and the source and target proxies
	 *
	 * @param subarchId string for the ID of the subarchitecture
	 * @param probExists the probability of the proxy
	 * @param features the features
	 * @param sourceProxy the source proxy
	 * @param targetProxy the target proxy
	 * @return the new relation proxy
	 */
    autogen::specialentities::RelationProxyPtr createNewRelationProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
						    float probExists,
						    const autogen::core::FeaturesList & features,
						    const autogen::core::FeatureValues source,
						    const autogen::core::FeatureValues target);


    /**
     * Create a new proxy given the ID of the originating subarchitecture,
     * the probability of the proxy, and a list of features
     *
     * @param subarchId string for the ID of the subarchitecture
     * @param probExists
     * @param features
     * @return
     */
    autogen::core::ProxyPtr createNewProxy (const cast::cdl::WorkingMemoryPointerPtr & origin,
					    float probExists,
					    const autogen::core::FeaturesList & features);

    /**
     * Add a new feature to the proxy (and regenerate the probability
     * distribution, given this new information)
     *
     * @param proxy the proxy
     * @param feat the feature to add
     */
    void addFeatureToProxy(autogen::core::ProxyPtr proxy,
			   autogen::core::FeaturePtr feat);



    /**
     * Create a new StringValue given a string and a probability
     *
     * @param val the string
     * @param prob the probability value
     * @return the StringValue
     */
    autogen::featvalues::StringValuePtr createStringValue(const std::string & val,
							  float prob);



    /**
     * Create a new IntegerValue given an integer and a probability
     *
     * @param val the integer
     * @param prob the probability value
     * @return the IntegerValue
     */
    autogen::featvalues::IntegerValuePtr  createIntegerValue(int val, float prob) ;




    /**
     * Create a new UnknownValue given a probability
     *
     * @param prob the probability value
     * @return the IntegerValue
     */
    autogen::featvalues::UnknownValuePtr  createUnknownValue(float prob) ;


    /**
     * Create a new BooleanValue given a boolean and a probability
     *
     * @param val the boolean
     * @param prob the probability value
     * @return the BooleanValue
     */
    autogen::featvalues::BooleanValuePtr  createBooleanValue(bool val, float prob) ;


    /**
     * Create a new feature, without feature values
     * @param featlabel the feature label
     * @return the new feature
     */
    autogen::core::FeaturePtr createFeature(const std::string & featlabel);

    /**
     * Create a new feature with a unique feature value
     * @param featlabel the feature label
     * @param featvalue the feature value
     * @return the new feature
     */
    autogen::core::FeaturePtr createFeatureWithUniqueFeatureValue
    (const std::string & featlabel, autogen::core::FeatureValuePtr featvalue);


    /**
     * Add a new feature value to an existing feature
     * @param feat the feature
     * @param featval the feature value
     */
    void addFeatureValueToFeature(autogen::core::FeaturePtr feat,
				  autogen::core::FeatureValuePtr featval);

    /**
     * Create a new feature containing several alternative feature values
     * @param featlabel the feature label
     * @param featvalues the array of feature values
     * @return the feature
     */
    autogen::core::FeaturePtr createFeatureWithAlternativeFeatureValues
    (const std::string & featlabel, const autogen::core::FeatureValues & featvalues);

    /**
     * Insert the proxy in the binder working memory
     * @param proxy the proxy
     */
    void addProxyToWM(autogen::core::ProxyPtr proxy);

    /**
     * Overwrite an existing proxy with a new one
     * (the new proxy needs to have the same entityID has the existing one)
     *
     * @param proxy the new proxy
     */
    void overwriteProxyInWM(autogen::core::ProxyPtr proxy);


    /**
     * Delete an existing proxy
     * @param proxy the proxy to delete
     */
    void deleteEntityInWM(autogen::core::ProxyPtr proxy);


     /**
     * Delete an existing proxy
     * @param _wma the address of the proxy to delete
     */
    void deleteEntityInWM(std::string _id);

  };

}

#endif
