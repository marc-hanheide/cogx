(define (problem strips-mprime-x-3)
   (:domain mystery-prime-strips)
   (:objects pepper pea lemon snickers marzipan popover melon orange
             chocolate ham mutton entertainment triumph love satisfaction
             laceration anxiety grief depression boils angina hangover
             jealousy goias guanabara bavaria arizona manitoba vulcan venus
             neptune - block )
   (:init
          (= (total-cost) 0)
          (food pepper)
          (food pea)
          (food lemon)
          (food snickers)
          (food marzipan)
          (food popover)
          (food melon)
          (food orange)
          (food chocolate)
          (food ham)
          (food mutton)
          (pleasure entertainment)
          (pleasure triumph)
          (pleasure love)
          (pleasure satisfaction)
          (pain laceration)
          (pain anxiety)
          (pain grief)
          (pain depression)
          (pain boils)
          (pain angina)
          (pain hangover)
          (pain jealousy)
          (province goias)
          (province guanabara)
          (province bavaria)
          (province arizona)
          (province manitoba)
          (planet vulcan)
          (planet venus)
          (planet neptune)
          (eats orange mutton)
          (harmony satisfaction neptune)
          (craves angina orange)
          (craves grief marzipan)
          (eats pea snickers)
          (eats popover pepper)
          (attacks guanabara bavaria)
          (eats mutton orange)
          (attacks bavaria arizona)
          (attacks goias guanabara)
          (harmony triumph neptune)
          (eats marzipan pea)
          (locale popover arizona)
          (locale ham manitoba)
          (eats mutton melon)
          (craves jealousy ham)
          (eats pea pepper)
          (eats pepper melon)
          (locale melon manitoba)
          (locale pea bavaria)
          (craves boils melon)
          (eats popover lemon)
          (craves laceration pepper)
          (eats popover marzipan)
          (eats chocolate ham)
          (locale mutton guanabara)
          (eats melon ham)
          (orbits venus neptune)
          (harmony love neptune)
          (eats marzipan snickers)
          (eats pepper popover)
          (locale lemon goias)
          (craves triumph popover)
          (orbits vulcan venus)
          (eats ham melon)
          (locale marzipan bavaria)
          (eats chocolate orange)
          (eats marzipan popover)
          (eats orange chocolate)
          (eats snickers pea)
          (eats pea marzipan)
          (harmony entertainment venus)
          (eats lemon pea)
          (eats melon pepper)
          (locale orange bavaria)
          (craves love orange)
          (craves anxiety snickers)
          (locale pepper manitoba)
          (eats melon mutton)
          (eats pepper pea)
          (locale chocolate guanabara)
          (eats ham chocolate)
          (craves entertainment pepper)
          (eats snickers marzipan)
          (attacks arizona manitoba)
          (craves satisfaction mutton)
          (eats lemon popover)
          (locale snickers bavaria)
          (eats pea lemon)
          (craves depression popover)
          (craves hangover chocolate))
   (:goal (and (craves hangover ham)))
    (:metric minimize (total-cost))
   )
