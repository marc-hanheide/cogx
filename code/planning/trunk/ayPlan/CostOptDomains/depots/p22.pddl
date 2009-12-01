(define (problem depotprob22) (:domain Depot)
(:objects
	depot0 depot1 depot2 depot3 depot4 depot5 - Depot
	distributor0 distributor1 distributor2 distributor3 distributor4 distributor5 - Distributor
	truck0 truck1 truck2 truck3 truck4 truck5 - Truck
	pallet0 pallet1 pallet2 pallet3 pallet4 pallet5 pallet6 pallet7 pallet8 pallet9 pallet10 pallet11 pallet12 pallet13 pallet14 pallet15 pallet16 pallet17 pallet18 pallet19 - Pallet
	crate0 crate1 crate2 crate3 crate4 crate5 crate6 crate7 crate8 crate9 crate10 crate11 crate12 crate13 crate14 crate15 crate16 crate17 crate18 crate19 - Crate
	hoist0 hoist1 hoist2 hoist3 hoist4 hoist5 hoist6 hoist7 hoist8 hoist9 hoist10 hoist11 hoist12 hoist13 hoist14 - Hoist)
(:init
    (= (total-cost) 0)
	(at pallet0 depot0)
	(clear crate10)
	(at pallet1 depot1)
	(clear crate17)
	(at pallet2 depot2)
	(clear pallet2)
	(at pallet3 depot3)
	(clear pallet3)
	(at pallet4 depot4)
	(clear pallet4)
	(at pallet5 depot5)
	(clear crate8)
	(at pallet6 distributor0)
	(clear pallet6)
	(at pallet7 distributor1)
	(clear crate19)
	(at pallet8 distributor2)
	(clear crate13)
	(at pallet9 distributor3)
	(clear crate16)
	(at pallet10 distributor4)
	(clear crate14)
	(at pallet11 distributor5)
	(clear crate0)
	(at pallet12 distributor4)
	(clear crate18)
	(at pallet13 distributor2)
	(clear crate15)
	(at pallet14 depot3)
	(clear pallet14)
	(at pallet15 depot3)
	(clear pallet15)
	(at pallet16 distributor1)
	(clear pallet16)
	(at pallet17 distributor2)
	(clear crate6)
	(at pallet18 distributor3)
	(clear pallet18)
	(at pallet19 distributor1)
	(clear pallet19)
	(at truck0 depot2)
	(at truck1 distributor4)
	(at truck2 distributor2)
	(at truck3 depot0)
	(at truck4 distributor3)
	(at truck5 distributor0)
	(at hoist0 depot0)
	(available hoist0)
	(at hoist1 depot1)
	(available hoist1)
	(at hoist2 depot2)
	(available hoist2)
	(at hoist3 depot3)
	(available hoist3)
	(at hoist4 depot4)
	(available hoist4)
	(at hoist5 depot5)
	(available hoist5)
	(at hoist6 distributor0)
	(available hoist6)
	(at hoist7 distributor1)
	(available hoist7)
	(at hoist8 distributor2)
	(available hoist8)
	(at hoist9 distributor3)
	(available hoist9)
	(at hoist10 distributor4)
	(available hoist10)
	(at hoist11 distributor5)
	(available hoist11)
	(at hoist12 depot0)
	(available hoist12)
	(at hoist13 depot5)
	(available hoist13)
	(at hoist14 depot5)
	(available hoist14)
	(at crate0 distributor5)
	(on crate0 pallet11)
	(at crate1 depot0)
	(on crate1 pallet0)
	(at crate2 distributor4)
	(on crate2 pallet10)
	(at crate3 distributor2)
	(on crate3 pallet8)
	(at crate4 distributor1)
	(on crate4 pallet7)
	(at crate5 distributor4)
	(on crate5 crate2)
	(at crate6 distributor2)
	(on crate6 pallet17)
	(at crate7 depot5)
	(on crate7 pallet5)
	(at crate8 depot5)
	(on crate8 crate7)
	(at crate9 depot0)
	(on crate9 crate1)
	(at crate10 depot0)
	(on crate10 crate9)
	(at crate11 distributor3)
	(on crate11 pallet9)
	(at crate12 depot1)
	(on crate12 pallet1)
	(at crate13 distributor2)
	(on crate13 crate3)
	(at crate14 distributor4)
	(on crate14 crate5)
	(at crate15 distributor2)
	(on crate15 pallet13)
	(at crate16 distributor3)
	(on crate16 crate11)
	(at crate17 depot1)
	(on crate17 crate12)
	(at crate18 distributor4)
	(on crate18 pallet12)
	(at crate19 distributor1)
	(on crate19 crate4)
)

(:goal (and
		(on crate0 pallet14)
		(on crate1 pallet15)
		(on crate2 pallet13)
		(on crate3 pallet18)
		(on crate5 pallet12)
		(on crate6 pallet6)
		(on crate7 pallet5)
		(on crate8 crate10)
		(on crate9 pallet17)
		(on crate10 crate17)
		(on crate11 pallet1)
		(on crate12 pallet16)
		(on crate13 crate16)
		(on crate15 pallet7)
		(on crate16 crate15)
		(on crate17 pallet2)
		(on crate18 pallet4)
		(on crate19 pallet9)
	)
)
(:metric minimize (total-cost))
)
