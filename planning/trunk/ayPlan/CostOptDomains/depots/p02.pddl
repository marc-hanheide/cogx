(define (problem depotprob02) (:domain Depot)
(:objects
	depot0 - Depot
	distributor0 distributor1 - Distributor
	truck0 truck1 - Truck
	pallet0 pallet1 pallet2 - Pallet
	crate0 crate1 crate2 crate3 - Crate
	hoist0 hoist1 hoist2 - Hoist)
(:init
    (= (total-cost) 0)
	(at hoist2 distributor1)
	(at hoist1 distributor0)
	(at hoist0 depot0)
	(at crate3 distributor0)
	(at crate2 distributor1)
	(at crate1 distributor1)
	(at crate0 depot0)
	(at pallet2 distributor1)
	(at pallet1 distributor0)
	(at pallet0 depot0)
	(at truck1 depot0)
	(at truck0 depot0)
	(available hoist2)
	(available hoist1)
	(available hoist0)
	(clear crate3)
	(clear crate2)
	(clear crate0)
	(on crate3 pallet1)
	(on crate2 crate1)
	(on crate1 pallet2)
	(on crate0 pallet0)
)

(:goal (and
		(on crate0 pallet2)
		(on crate1 crate3)
		(on crate2 pallet0)
		(on crate3 pallet1)
	)
)
(:metric minimize (total-cost))
)
