#include "ExampleProxyPusher.hpp"


extern "C" {
  cast::interfaces::CASTComponentPtr
  newComponent() {
    return new ExampleProxyPusher();
  }
}



using namespace binder::autogen::core;
using namespace binder::autogen::featvalues;
using namespace cast;
using namespace cast::cdl;


void ExampleProxyPusher::start() {
  //must call super start to ensure that the reader sets up change
  //filters
  BindingWorkingMemoryReader::start();
}

void ExampleProxyPusher::runComponent() {
  sleepComponent(1000);
  println("faking stuff up");

  //copied from fake visual thingy

  cast::cdl::WorkingMemoryPointerPtr origin1 = new cast::cdl::WorkingMemoryPointer();
  origin1->address.subarchitecture = "no";
  origin1->address.id = "local";
  origin1->type = "data"; //uh oh, do we always need to include this?

  ProxyPtr proxy1 = createNewProxy(origin1, 1.0f);

  FeaturePtr feat1a = new Feature();
  feat1a->featlabel = "category";
  feat1a->alternativeValues.push_back(new
				   binder::autogen::featvalues::StringValue(1, getCASTTime(), "roboeeeet"));
  addFeatureToProxy(proxy1, feat1a);

  addProxyToWM(proxy1);


  WorkingMemoryPointerPtr origin2 = createWorkingMemoryPointer ("spatial.sa", "bong", "place");
  ProxyPtr proxy2 = createNewProxy (origin2, 1.0f);

  FeatureValuePtr zero = createStringValue ("0" , 1.0f);
  FeaturePtr feat2a = createFeatureWithUniqueFeatureValue("place.id", zero);
  addFeatureToProxy (proxy2, feat2a);

  FeatureValuePtr explored = createStringValue ("true" , 1.0f);
  FeaturePtr feat2b = createFeatureWithUniqueFeatureValue("explored", explored);
  addFeatureToProxy (proxy2, feat2b);

  addProxyToWM(proxy2);

   sleepComponent(1000);


  cast::cdl::WorkingMemoryPointerPtr origin3 = new cast::cdl::WorkingMemoryPointer();
    origin3->address.subarchitecture = "no";
    origin3->address.id = "local";
    origin3->type = "data"; //uh oh, do we always need to include this?

    FeaturePtr source3 = new Feature();
      source3->featlabel = "source";
      source3->alternativeValues.push_back(new
  			   binder::autogen::featvalues::AddressValue(1, getCASTTime(), proxy1->entityID));

      FeaturePtr target3 = new Feature();
      target3->featlabel = "target";
      target3->alternativeValues.push_back(new
     			   binder::autogen::featvalues::AddressValue(1, getCASTTime(), proxy2->entityID));

      ProxyPtr proxy3 = createNewRelationProxy(origin3, 1.0f, source3->alternativeValues, target3->alternativeValues);

    FeaturePtr feat3a = new Feature();
    feat3a->featlabel = "located";
    feat3a->alternativeValues.push_back(new
  				   binder::autogen::featvalues::StringValue(1, getCASTTime(), "true"));
    addFeatureToProxy(proxy3, feat3a);

    proxy3->entityID = proxy1->entityID;
    overwriteProxyInWM(proxy3);




 /**   sleepComponent(1000);

    WorkingMemoryPointerPtr origin4 = createWorkingMemoryPointer ("spatial.sa", "bong", "place");
    ProxyPtr proxy4 = createNewProxy (origin4, 1.0f);

    FeatureValuePtr one = createStringValue ("1" , 1.0f);
    FeaturePtr feat4a = createFeatureWithUniqueFeatureValue("place.id", one);
    addFeatureToProxy (proxy4, feat4a);

    FeatureValuePtr explored2 = createStringValue ("trsasue" , 1.0f);
    FeaturePtr feat4b = createFeatureWithUniqueFeatureValue("explored", explored2);
    addFeatureToProxy (proxy4, feat4b);

 //   addProxyToWM(proxy4);

    sleepComponent(1000);

    proxy4->entityID = proxy1->entityID;
    overwriteProxyInWM(proxy4);*/



   //  sleepComponent(1000);
/**
    cast::cdl::WorkingMemoryPointerPtr origin5 = new cast::cdl::WorkingMemoryPointer();
      origin5->address.subarchitecture = "no";
      origin5->address.id = "local";
      origin5->type = "data"; //uh oh, do we always need to include this?

      FeaturePtr source5 = new Feature();
        source5->featlabel = "source";
        source5->alternativeValues.push_back(new
    			   binder::autogen::featvalues::AddressValue(1, getCASTTime(), proxy2->entityID));

        FeaturePtr target5 = new Feature();
        target5->featlabel = "target";
        target5->alternativeValues.push_back(new
       			   binder::autogen::featvalues::AddressValue(1, getCASTTime(), proxy4->entityID));

        ProxyPtr proxy5 = createNewRelationProxy(origin5, 1.0f, source5->alternativeValues, target5->alternativeValues);

      FeaturePtr feat5a = new Feature();
      feat5a->featlabel = "connected";
      feat5a->alternativeValues.push_back(new
    				   binder::autogen::featvalues::StringValue(1, getCASTTime(), "true"));
      addFeatureToProxy(proxy5, feat5a);


      addProxyToWM(proxy5);




   //   sleepComponent(1000);

      cast::cdl::WorkingMemoryPointerPtr origin6 = new cast::cdl::WorkingMemoryPointer();
        origin6->address.subarchitecture = "no";
        origin6->address.id = "local";
        origin6->type = "data"; //uh oh, do we always need to include this?

        FeaturePtr source6 = new Feature();
          source6->featlabel = "source";
          source6->alternativeValues.push_back(new
      			   binder::autogen::featvalues::AddressValue(1, getCASTTime(), proxy4->entityID));

          FeaturePtr target6 = new Feature();
          target6->featlabel = "target";
          target6->alternativeValues.push_back(new
         			   binder::autogen::featvalues::AddressValue(1, getCASTTime(), proxy2->entityID));

          ProxyPtr proxy6 = createNewRelationProxy(origin6, 1.0f, source6->alternativeValues, target6->alternativeValues);

        FeaturePtr feat6a = new Feature();
        feat6a->featlabel = "connected";
        feat6a->alternativeValues.push_back(new
      				   binder::autogen::featvalues::StringValue(1, getCASTTime(), "true"));
        addFeatureToProxy(proxy6, feat6a);


        addProxyToWM(proxy6);



    //    sleepComponent(1000);

        cast::cdl::WorkingMemoryPointerPtr origin7 = new cast::cdl::WorkingMemoryPointer();
          origin7->address.subarchitecture = "no";
          origin7->address.id = "local";
          origin7->type = "data"; //uh oh, do we always need to include this?

          FeaturePtr source7 = new Feature();
            source7->featlabel = "source";
            source7->alternativeValues.push_back(new
        			   binder::autogen::featvalues::AddressValue(1, getCASTTime(), proxy1->entityID));

            FeaturePtr target7 = new Feature();
            target7->featlabel = "target";
            target7->alternativeValues.push_back(new
           			   binder::autogen::featvalues::AddressValue(1, getCASTTime(), proxy4->entityID));

            ProxyPtr proxy7 = createNewRelationProxy(origin7, 1.0f, source7->alternativeValues, target7->alternativeValues);

          FeaturePtr feat7a = new Feature();
          feat7a->featlabel = "connected";
          feat7a->alternativeValues.push_back(new
        				   binder::autogen::featvalues::StringValue(1, getCASTTime(), "true"));
          addFeatureToProxy(proxy7, feat7a);


          addProxyToWM(proxy7);

*/

}
