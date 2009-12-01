(define (problem FreeCell-07)
(:domain freecell)
(:objects
	diamond club heart spade - suitsort
	N0 N1 N2 N3 N4 N5 N6 N7 - denomination
	heart2 - card
	heart7 - card
	diamond7 - card
	spadeA - card
	diamond6 - card
	spade5 - card
	heart4 - card
	heartA - card
	club5 - card
	spade7 - card
	diamond5 - card
	club6 - card
	diamond4 - card
	spade3 - card
	diamondA - card
	spade6 - card
	spade4 - card
	club2 - card
	club4 - card
	heart3 - card
	club3 - card
	diamond3 - card
	club7 - card
	diamond2 - card
	heart6 - card
	heart5 - card
	spade2 - card
	clubA - card
	diamond0 - card
	club0 - card
	heart0 - card
	spade0 - card
	
)
(:init
    (= (total-cost) 0)
	(successor N1 N0)
	(successor N2 N1)
	(successor N3 N2)
	(successor N4 N3)
	(successor N5 N4)
	(successor N6 N5)
	(successor N7 N6)
	(cellspace N4)
	(clear heart2)
	(on heart2 spade5)
	(on spade5 diamond5)
	(on diamond5 spade6)
	(on spade6 club3)
	(on club3 heart5)
	(bottomcol heart5)
	(clear heart7)
	(on heart7 heart4)
	(on heart4 club6)
	(on club6 spade4)
	(on spade4 diamond3)
	(on diamond3 spade2)
	(bottomcol spade2)
	(clear diamond7)
	(on diamond7 heartA)
	(on heartA diamond4)
	(on diamond4 club2)
	(on club2 club7)
	(on club7 clubA)
	(bottomcol clubA)
	(clear spadeA)
	(on spadeA club5)
	(on club5 spade3)
	(on spade3 club4)
	(on club4 diamond2)
	(bottomcol diamond2)
	(clear diamond6)
	(on diamond6 spade7)
	(on spade7 diamondA)
	(on diamondA heart3)
	(on heart3 heart6)
	(bottomcol heart6)
	(colspace N0)
	(value heart2 N2)
	(suit heart2 heart)
	(canstack heart2 club3)
	(canstack heart2 spade3)
	(value heart7 N7)
	(suit heart7 heart)
	(value diamond7 N7)
	(suit diamond7 diamond)
	(value spadeA N1)
	(suit spadeA spade)
	(canstack spadeA diamond2)
	(canstack spadeA heart2)
	(value diamond6 N6)
	(suit diamond6 diamond)
	(canstack diamond6 club7)
	(canstack diamond6 spade7)
	(value spade5 N5)
	(suit spade5 spade)
	(canstack spade5 diamond6)
	(canstack spade5 heart6)
	(value heart4 N4)
	(suit heart4 heart)
	(canstack heart4 club5)
	(canstack heart4 spade5)
	(value heartA N1)
	(suit heartA heart)
	(canstack heartA club2)
	(canstack heartA spade2)
	(value club5 N5)
	(suit club5 club)
	(canstack club5 diamond6)
	(canstack club5 heart6)
	(value spade7 N7)
	(suit spade7 spade)
	(value diamond5 N5)
	(suit diamond5 diamond)
	(canstack diamond5 club6)
	(canstack diamond5 spade6)
	(value club6 N6)
	(suit club6 club)
	(canstack club6 diamond7)
	(canstack club6 heart7)
	(value diamond4 N4)
	(suit diamond4 diamond)
	(canstack diamond4 club5)
	(canstack diamond4 spade5)
	(value spade3 N3)
	(suit spade3 spade)
	(canstack spade3 diamond4)
	(canstack spade3 heart4)
	(value diamondA N1)
	(suit diamondA diamond)
	(canstack diamondA club2)
	(canstack diamondA spade2)
	(value spade6 N6)
	(suit spade6 spade)
	(canstack spade6 diamond7)
	(canstack spade6 heart7)
	(value spade4 N4)
	(suit spade4 spade)
	(canstack spade4 diamond5)
	(canstack spade4 heart5)
	(value club2 N2)
	(suit club2 club)
	(canstack club2 diamond3)
	(canstack club2 heart3)
	(value club4 N4)
	(suit club4 club)
	(canstack club4 diamond5)
	(canstack club4 heart5)
	(value heart3 N3)
	(suit heart3 heart)
	(canstack heart3 club4)
	(canstack heart3 spade4)
	(value club3 N3)
	(suit club3 club)
	(canstack club3 diamond4)
	(canstack club3 heart4)
	(value diamond3 N3)
	(suit diamond3 diamond)
	(canstack diamond3 club4)
	(canstack diamond3 spade4)
	(value club7 N7)
	(suit club7 club)
	(value diamond2 N2)
	(suit diamond2 diamond)
	(canstack diamond2 club3)
	(canstack diamond2 spade3)
	(value heart6 N6)
	(suit heart6 heart)
	(canstack heart6 club7)
	(canstack heart6 spade7)
	(value heart5 N5)
	(suit heart5 heart)
	(canstack heart5 club6)
	(canstack heart5 spade6)
	(value spade2 N2)
	(suit spade2 spade)
	(canstack spade2 diamond3)
	(canstack spade2 heart3)
	(value clubA N1)
	(suit clubA club)
	(canstack clubA diamond2)
	(canstack clubA heart2)
	(home diamond0)
	(value diamond0 N0)
	(suit diamond0 diamond)
	(home club0)
	(value club0 N0)
	(suit club0 club)
	(home heart0)
	(value heart0 N0)
	(suit heart0 heart)
	(home spade0)
	(value spade0 N0)
	(suit spade0 spade)
)
(:goal (and
	(home diamond7)
	(home club7)
	(home heart7)
	(home spade7)
))
	(:metric minimize (total-cost))
)
