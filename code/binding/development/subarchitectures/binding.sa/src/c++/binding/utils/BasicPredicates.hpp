#ifndef BINDING_BASIC_PREDICATES_HPP_
#define BINDING_BASIC_PREDICATES_HPP_
//#include <binding/utils/LocalClasses.hpp>
#include <binding/utils/LocalClassesSets.hpp>
//#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <functional>
#include <deque>
#include <boost/function.hpp>
#include <boost/regex.hpp>
#include <boost/ref.hpp>
#include <map>

namespace Binding {

/// tests sth about proxies or unions
template<class LocalBindingDataPtrT>
struct AbstractPredicate 
  : public std::unary_function<const LocalBindingDataPtrT&, bool>
{
  typedef LocalBindingDataPtrT local_ptr_type;
  typedef AbstractPredicate<LocalBindingDataPtrT> abstract_predicate_type;
  typedef boost::shared_ptr<AbstractPredicate<LocalBindingDataPtrT> > abstract_predicate_ptr;
  
  virtual bool operator()(const std::pair<const std::string,LocalBindingDataPtrT>& _pair) const {
    return this->test(_pair.second);
  }

  virtual bool test(const LocalBindingDataPtrT&) const = 0;
  
  virtual abstract_predicate_ptr clone() const = 0;
  
  virtual ~AbstractPredicate(){}
  
  virtual void printtype() {std::cout << typeid(*this).name();}
protected:
  AbstractPredicate(const AbstractPredicate&){}
  AbstractPredicate(){}
private:
  const AbstractPredicate& operator=(const AbstractPredicate&);
};

template<class Predicate>
typename Predicate::abstract_predicate_ptr
clone_predicate(const Predicate& _pred) 
{
  return typename Predicate::abstract_predicate_ptr(new Predicate(_pred));
}

/// always true
template<class LocalBindingDataPtrT>
struct TruePredicate : public AbstractPredicate<LocalBindingDataPtrT> {
  /// always returns true
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return true;
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr 
  clone() const {return clone_predicate(*this);}
};



template<class LocalBindingDataPtrT>
struct UnaryPredicate : public AbstractPredicate<LocalBindingDataPtrT> {
  UnaryPredicate(const AbstractPredicate<LocalBindingDataPtrT>& _pred) : pred(_pred.clone()) {}
  boost::shared_ptr<AbstractPredicate<LocalBindingDataPtrT> > pred;
protected:
  virtual ~UnaryPredicate() {};
};

template<class LocalBindingDataPtrT>
struct BinaryPredicate : public AbstractPredicate<LocalBindingDataPtrT> {
  BinaryPredicate(const AbstractPredicate<LocalBindingDataPtrT>& _pred1,
		  const AbstractPredicate<LocalBindingDataPtrT>& _pred2) 
    : predicate_1(_pred1.clone()),
      predicate_2(_pred2.clone()){}
  boost::shared_ptr<AbstractPredicate<LocalBindingDataPtrT> > predicate_1;
  boost::shared_ptr<AbstractPredicate<LocalBindingDataPtrT> > predicate_2;
protected:
  virtual ~BinaryPredicate() {};
};


template<class LocalBindingDataPtrT>
struct NegatedPredicate : public UnaryPredicate<LocalBindingDataPtrT> {
  NegatedPredicate(const AbstractPredicate<LocalBindingDataPtrT>& _pred) : 
    UnaryPredicate<LocalBindingDataPtrT>(_pred) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return !(this->pred->test(_ptr));
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr 
  clone() const {return clone_predicate(*this);}
};

/// convenience factory function for NegatedPredicate
inline
NegatedPredicate<ProxyPtr>
operator!(const AbstractPredicate<ProxyPtr>& _pred)
{
  return NegatedPredicate<ProxyPtr>(_pred);
}

/// convenience factory function for NegatedPredicate
inline
NegatedPredicate<UnionPtr>
operator!(const AbstractPredicate<UnionPtr>& _pred)
{
  return NegatedPredicate<UnionPtr>(_pred);
}

template<class LocalBindingDataPtrT>
struct AndPredicate : public BinaryPredicate<LocalBindingDataPtrT> {
  AndPredicate(const AbstractPredicate<LocalBindingDataPtrT>& _predicate_1,
	       const AbstractPredicate<LocalBindingDataPtrT>& _predicate_2) 
    : BinaryPredicate<LocalBindingDataPtrT>(_predicate_1, _predicate_2) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return this->predicate_1->test(_ptr) && this->predicate_2->test(_ptr);
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr 
  clone() const {return clone_predicate(*this);}
};

/// convenience factory function for AndPredicate
inline
AndPredicate<ProxyPtr>
operator&& (const AbstractPredicate<ProxyPtr>& _predicate_1,
	    const AbstractPredicate<ProxyPtr>& _predicate_2) 
{
  return AndPredicate<ProxyPtr>(_predicate_1,_predicate_2);
}

/// convenience factory function for AndPredicate
inline
AndPredicate<UnionPtr>
operator&& (const AbstractPredicate<UnionPtr>& _predicate_1,
	    const AbstractPredicate<UnionPtr>& _predicate_2) 
{
  return AndPredicate<UnionPtr>(_predicate_1,_predicate_2);
}

template<class LocalBindingDataPtrT>
struct InclusiveOrPredicate : public BinaryPredicate<LocalBindingDataPtrT> {

  InclusiveOrPredicate(const AbstractPredicate<LocalBindingDataPtrT>& _predicate_1,
	       const AbstractPredicate<LocalBindingDataPtrT>& _predicate_2) 
    : BinaryPredicate<LocalBindingDataPtrT>(_predicate_1, _predicate_2) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return this->predicate_1->test(_ptr) || this->predicate_2->test(_ptr);
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr 
  clone() const {return clone_predicate(*this);}

};

/// convenience factory function for InclusiveOrPredicate
inline
InclusiveOrPredicate<ProxyPtr>
operator|| (const AbstractPredicate<ProxyPtr>& _predicate_1,
	    const AbstractPredicate<ProxyPtr>& _predicate_2) 
{
  return InclusiveOrPredicate<ProxyPtr>(_predicate_1,_predicate_2);
}

/// convenience factory function for InclusiveOrPredicate
inline
InclusiveOrPredicate<UnionPtr>
operator|| (const AbstractPredicate<UnionPtr>& _predicate_1,
	    const AbstractPredicate<UnionPtr>& _predicate_2) 
{
  return InclusiveOrPredicate<UnionPtr>(_predicate_1,_predicate_2);
}

template<class LocalBindingDataPtrT>
struct ImplicationPredicate : public BinaryPredicate<LocalBindingDataPtrT> {
  ImplicationPredicate(const AbstractPredicate<LocalBindingDataPtrT>& _predicate_1,
	       const AbstractPredicate<LocalBindingDataPtrT>& _predicate_2) 
    : BinaryPredicate<LocalBindingDataPtrT>(_predicate_1, _predicate_2) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return !this->predicate_1->test(_ptr) || this->predicate_2->test(_ptr);
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// convenience factory function for ImplicationPredicate (operator-> does not work...)
inline
ImplicationPredicate<ProxyPtr>
operator->* (const AbstractPredicate<ProxyPtr>& _predicate_1,
	     const AbstractPredicate<ProxyPtr>& _predicate_2) 
{
  return ImplicationPredicate<ProxyPtr>(_predicate_1,_predicate_2);
}

 
/// convenience factory function for ImplicationPredicate (operator-> does not work...)
inline
ImplicationPredicate<UnionPtr>
operator->* (const AbstractPredicate<UnionPtr>& _predicate_1,
	     const AbstractPredicate<UnionPtr>& _predicate_2) 
{
  return ImplicationPredicate<UnionPtr>(_predicate_1,_predicate_2);
}

template<class LocalBindingDataPtrT>
struct EquivalencePredicate : public BinaryPredicate<LocalBindingDataPtrT> {
  EquivalencePredicate(const AbstractPredicate<LocalBindingDataPtrT>& _predicate_1,
		       const AbstractPredicate<LocalBindingDataPtrT>& _predicate_2) 
    : BinaryPredicate<LocalBindingDataPtrT>(_predicate_1, _predicate_2) {}
  
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return this->predicate_1->test(_ptr) == this->predicate_2->test(_ptr);
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}
};

/// convenience factory function for EquivalencePredicate
inline
EquivalencePredicate<ProxyPtr>
operator== (const AbstractPredicate<ProxyPtr>& _predicate_1,
	     const AbstractPredicate<ProxyPtr>& _predicate_2) 
{
  return EquivalencePredicate<ProxyPtr>(_predicate_1,_predicate_2);
}

 
/// convenience factory function for EquivalencePredicate
inline
EquivalencePredicate<UnionPtr>
operator== (const AbstractPredicate<UnionPtr>& _predicate_1,
	     const AbstractPredicate<UnionPtr>& _predicate_2) 
{
  return EquivalencePredicate<UnionPtr>(_predicate_1,_predicate_2);
}

/// returns same as predicate... just a wrapper to avoid problems with
/// stl algorithms (the reference to reference problem)
template<class LocalBindingDataPtrT>
struct IdentityPredicate : public UnaryPredicate<LocalBindingDataPtrT> {
  IdentityPredicate(const AbstractPredicate<LocalBindingDataPtrT>& _pred) : 
    UnaryPredicate<LocalBindingDataPtrT>(_pred) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return this->pred->test(_ptr);
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// convenience factory function for IdentityPredicate
inline
IdentityPredicate<ProxyPtr>
wrap(const AbstractPredicate<ProxyPtr>& _pred)
{
  return IdentityPredicate<ProxyPtr>(_pred);
}

/// convenience factory function for IdentityPredicate
inline
IdentityPredicate<UnionPtr>
wrap(const AbstractPredicate<UnionPtr>& _pred)
{
  return IdentityPredicate<UnionPtr>(_pred);
}

/// SetT either a ProxySet or UnionSet, returns true if all in a set has a propery
template<typename LocalBindingDataPtrT>
bool 
true_for_all(const typename LocalDataSets<LocalBindingDataPtrT>::set& _set, 
	     const AbstractPredicate<LocalBindingDataPtrT>& _pr)
{
  // returns true if none of elements match negated predicate
  return std::find_if(_set.begin(),_set.end(),!_pr) == _set.end();
};

/// SetT either a ProxySet or UnionSet, returns true if at least one
/// element has the property
template<typename LocalBindingDataPtrT>
bool 
true_for_some(const typename LocalDataSets<LocalBindingDataPtrT>::set& _set, 
		   const AbstractPredicate<LocalBindingDataPtrT>& _pr)
{
  return find_if(_set.begin(),_set.end(),wrap(_pr)) != _set.end();
};

} // namespace Binding

#endif //BINDING_BASIC_PREDICATES_HPP_
