
/// \c MyComponent is explained here
class MyComponent : public cast::PrivilegedManagedProcess {
public:
  /// The constructor...
  MyComponent(const string& _id ///< \p  explained here
	      );
  
  /// a member function should be explained
  cast::cdl::WorkingMemoryAddress where_am_i() const;
  /// \callgraph
  /// \callergraph
};

/// \c MyClass is explained here 
class MyClass {
public:
  /// The constructor...
  MyClass(const string& _id ///< \p  explained here
	  );
  /// f is a function
  int f();

  MyComponent m_component_member;
};

/// MyMonitor is explained here
class MyMonitor : public Binding::AbstractMonitor {
public:
  /// The constructor...
  MyMonitor(const string& _id///< \p  explained here
	    );
  
  /// a member function should be explained
  /// \callgraph
  /// \callergraph
  BindingData::FeaturePointer where_is_my_feature() const {
    example::MyComponent::where_am_i(); // test of calling a function...
  }
};

