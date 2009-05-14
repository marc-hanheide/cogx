#ifndef HOG_VISITOR_H
#define HOG_VISITOR_H

namespace HOG{

	// Visitors

	struct false_tag {};
	struct true_tag {};

	// visitor events
	// if you add new event, add it to the class HOG_visitor (see below, in this file)
	struct on_no_event {};
	struct on_examine_descriptor {};
	struct on_visited_location {};
	struct on_new_scale {};
	
	struct HOG_null_visitor{
		typedef on_no_event event_filter;

		template <class T>
		void operator()(T) { }
	};

	namespace detail {
		template <class Visitor, class T>
		inline void invoke_dispatch(Visitor& v, T x, HOG::true_tag) {
		   v(x);
		}
		template <class Visitor, class T>
		inline void invoke_dispatch(Visitor&, T, HOG::false_tag) { }

		template <class U, class V> struct is_same { typedef HOG::false_tag is_same_tag; };
		template <class U>          struct is_same<U, U> { typedef HOG::true_tag is_same_tag; };
	}

	// for one visitor
	template <class Visitor, class T, class Tag>
	inline void invoke_visitors(Visitor& v, T x, Tag) {
		typedef typename Visitor::event_filter Category;
		typedef typename detail::is_same<Category, Tag>::is_same_tag IsSameTag;
		detail::invoke_dispatch(v, x, IsSameTag());
	}

	// for a list of visitors:
	template <class Visitor, class Rest, class T, class Tag>
	inline void invoke_visitors(std::pair<Visitor, Rest>& vlist, T x, Tag tag) {
		typedef typename Visitor::event_filter Category;
		typedef typename detail::is_same<Category, Tag>::is_same_tag IsSameTag;

		detail::invoke_dispatch(vlist.first, x, IsSameTag());
		invoke_visitors(vlist.second, x, tag);
	}


	// HOG - visitors
	namespace Visitor { // define types that are passed to visitors
		struct ScaledImageParam{
			float scale;
			unsigned char* image;
			size_t width;
			size_t height;
			size_t widthstep;
		};
	}

	template<typename Visitors=HOG_null_visitor>
	struct HOG_visitor {
		HOG_visitor(Visitors vis):vis_(vis) { }
		// map algorithm-visitor calls to visitors (functors)

		// for each descriptor computed
		void examine_descriptor(const float* desc) {
			invoke_visitors(vis_, desc, on_examine_descriptor());
		}

		// after examine_descriptor: be careful these coordinates are NOT scaled
		void visited_location(std::pair<int,int> coords) {
			invoke_visitors(vis_, coords, on_visited_location());
		}

		// when scale is changed
		void new_scale(float scale, unsigned char* image, size_t width, size_t height, size_t widthstep) {
			Visitor::ScaledImageParam sp = {scale,image,width,height,widthstep};
			invoke_visitors(vis_,sp,on_new_scale());
		}

		Visitors vis_;
	};

	template <class Visitors>
	HOG_visitor<Visitors> make_HOG_visitor(Visitors vis) {
	  return HOG_visitor<Visitors>(vis);
	}

	typedef HOG_visitor<> default_HOG_visitor;


}

#endif
