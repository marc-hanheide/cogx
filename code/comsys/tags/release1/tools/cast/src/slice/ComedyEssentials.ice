#include <CDL.ice>

module comedyarch {

  module autogen {

   sequence < string > StringSeq;

    /**
     * Definition of component languages.
     */
    enum Funny {
      HAHA,
      PECULIAR
    };
    

    class Joke {
    };  

    class OneLiner extends Joke {
      string punchline;
    };

    class TwoLiner extends OneLiner {
      string setup;
    };
    
    
   sequence < Joke > JokeList;
    
    class JokeBook {
      string title;
      long jokeCount;
      JokeList jokes;
    };
    
    /**
     * These are the possible things the director can do.
     */
    enum DirectorActionType {
      AskTheAudience, CheckTheReaction
    };
    
    /**
     * This is how they're suggested to the director.
     */
    class DirectorAction {
      DirectorActionType action;
      cast::cdl::WorkingMemoryAddress address;
    };
    
    class Reaction {
    	  string react;
    };  

    interface TheGreatPretender {
    	      void getLies();
    };

  };
  
};
