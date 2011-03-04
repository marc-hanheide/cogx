% Container-in-room
tableLambda_kitchen = computeExclusive(0.8, [], []);
tableLambda_office = computeExclusive(0.3, [], []);
tableLambda_living_room = computeExclusive(0.7, [], []);
tableLambda_corridor = computeExclusive(0.02, [], []);

bookcaseLambda_kitchen = computeExclusive(0.1, [], []);
bookcaseLambda_office = computeExclusive(0.7, [], []);
bookcaseLambda_living_room = computeExclusive(0.8, [], []);
bookcaseLambda_corridor = computeExclusive(0.5, [], []);

cupboardLambda_kitchen = computeExclusive(0.9, [], []);
cupboardLambda_office = computeExclusive(0.1, [], []);
cupboardLambda_living_room = computeExclusive(0.1, [], []);
cupboardLambda_corridor = computeExclusive(0.4, [], []);

boxLambda_kitchen = computeExclusive(0.2, [], []);
boxLambda_office = computeExclusive(0.6, [], []);
boxLambda_living_room = computeExclusive(0, [], []);
boxLambda_corridor = computeExclusive(0.1, [], []);

dishwasherLambda_kitchen = computeExclusive(0.7, [], []);
dishwasherLambda_office = computeExclusive(0, [], []);
dishwasherLambda_living_room = computeExclusive(0, [], []);
dishwasherLambda_corridor = computeExclusive(0, [], []);

% Mug-in/on-object
mugOnTableLambda_kitchen = computeExclusive(0.4, [], []);
mugOnTableLambda_office = computeExclusive(0.1, [], []);
mugOnTableLambda_living_room = computeExclusive(0.3, [], []);
mugOnTableLambda_corridor = computeExclusive(0.1, [], []);

mugInBookcaseLambda_kitchen = computeExclusive(0, [], []);
mugInBookcaseLambda_office = computeExclusive(0.2, [], []);
mugInBookcaseLambda_living_room = computeExclusive(0.2, [], []);
mugInBookcaseLambda_corridor = computeExclusive(0.05, [], []);

mugInCupboardLambda_kitchen = computeExclusive(0.9, [], []);
mugInCupboardLambda_office = computeExclusive(0.9, [], []);
mugInCupboardLambda_living_room = computeExclusive(0.9, [], []);
mugInCupboardLambda_corridor = computeExclusive(0.9, [], []);

mugInBoxLambda_kitchen = computeExclusive(0.2, [], []);
mugInBoxLambda_office = computeExclusive(0.1, [], []);
mugInBoxLambda_living_room = computeExclusive(0, [], []);
mugInBoxLambda_corridor = computeExclusive(0.1, [], []);

mugInDishwasherLambda_kitchen = computeExclusive(0.7, [], []);
mugInDishwasherLambda_office = computeExclusive(0, [], []);
mugInDishwasherLambda_living_room = computeExclusive(0, [], []);
mugInDishwasherLambda_corridor = computeExclusive(0, [], []);

% Cereal-in/on-object
cerealOnTableLambda_kitchen = computeExclusive(0.7, [], []);
cerealOnTableLambda_office = computeExclusive(0.1, [], []);
cerealOnTableLambda_living_room = computeExclusive(0.2, [], []);
cerealOnTableLambda_corridor = computeExclusive(0.1, [], []);

cerealInBookcaseLambda_kitchen = computeExclusive(0.1, [], []);
cerealInBookcaseLambda_office = computeExclusive(0.05, [], []);
cerealInBookcaseLambda_living_room = computeExclusive(0.05, [], []);
cerealInBookcaseLambda_corridor = computeExclusive(0, [], []);

cerealInCupboardLambda_kitchen = computeExclusive(0.7, [], []);
cerealInCupboardLambda_office = computeExclusive(0.3, [], []);
cerealInCupboardLambda_living_room = computeExclusive(0.3, [], []);
cerealInCupboardLambda_corridor = computeExclusive(0.2, [], []);

cerealInBoxLambda_kitchen = computeExclusive(0, [], []);
cerealInBoxLambda_office = computeExclusive(0, [], []);
cerealInBoxLambda_living_room = computeExclusive(0, [], []);
cerealInBoxLambda_corridor = computeExclusive(0, [], []);

cerealInDishwasherLambda_kitchen = computeExclusive(0, [], []);
cerealInDishwasherLambda_office = computeExclusive(0, [], []);
cerealInDishwasherLambda_living_room = computeExclusive(0, [], []);
cerealInDishwasherLambda_corridor = computeExclusive(0, [], []);

% Book-in/on-object
bookOnTableLambda_kitchen = computeExclusive(0.1, [], []);
bookOnTableLambda_office = computeExclusive(0.6, [], []);
bookOnTableLambda_living_room = computeExclusive(0.7, [], []);
bookOnTableLambda_corridor = computeExclusive(0.5, [], []);

bookInBookcaseLambda_kitchen = computeExclusive(0.9, [], []);
bookInBookcaseLambda_office = computeExclusive(0.9, [], []);
bookInBookcaseLambda_living_room = computeExclusive(0.9, [], []);
bookInBookcaseLambda_corridor = computeExclusive(0.9, [], []);

bookInCupboardLambda_kitchen = computeExclusive(0, [], []);
bookInCupboardLambda_office = computeExclusive(0.1, [], []);
bookInCupboardLambda_living_room = computeExclusive(0, [], []);
bookInCupboardLambda_corridor = computeExclusive(0, [], []);

bookInBoxLambda_kitchen = computeExclusive(0, [], []);
bookInBoxLambda_office = computeExclusive(0, [], []);
bookInBoxLambda_living_room = computeExclusive(0, [], []);
bookInBoxLambda_corridor = computeExclusive(0, [], []);

bookInDishwasherLambda_kitchen = computeExclusive(0, [], []);
bookInDishwasherLambda_office = computeExclusive(0, [], []);
bookInDishwasherLambda_living_room = computeExclusive(0, [], []);
bookInDishwasherLambda_corridor = computeExclusive(0, [], []);

% Mug-in-room
mugInKitchenLambda = computeExclusive(0.8, ...
    [tableLambda_kitchen ...
    cupboardLambda_kitchen ...
    bookcaseLambda_kitchen ...
    boxLambda_kitchen ...
    dishwasherLambda_kitchen], ...
    [mugOnTableLambda_kitchen ...
    mugInCupboardLambda_kitchen ...
    mugInBookcaseLambda_kitchen ...
    mugInBoxLambda_kitchen ...
    mugInDishwasherLambda_kitchen]);

%etc
