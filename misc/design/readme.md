# Car Prototype Design
The files included in this folder are the design files for the car hardware. We used these cars as a prototype to show off our algorithms.

We were first planning such that the car carries the racks and moves around while carrying it. However, and due to lack of time and health, this approach has some **challenges**, we shall mention some:

- You have to make sure that the car doesn't hit the rack legs while getting beneath it. This seems a trivial task, but remember, we need precision as well as accuracy throughout the lifetime of the warehouse operation. Meaning that when the rack gets transported multiple times it still gets to the same central position in the grid, such that the next time a new robot comes to lift it, it doesn't hit the rack's legs.

- The rack has to either:
    - Rotate while the car rotates. Here you need some kind of locking mechanism between the rack and the car. Our initial design `(V_0.1)` clearly has a circle shape which doesn't help at all! This mechanism has to be smart enough to give some flexibility (the rack doesn't have to be above the car lift tool exactly) and tightness (the rack should not move around while being carried over the car). This can be achieved using more complex motors that can lift more weight and make the rack heavy and has a rough surface such that it rotates with the car. Other solutions can be figured out, but we didn't have enough time.
    - Stay stationary (i.e. same orientation) as the car rotates beneath it. I guess we need some decoupler to separate the movement of the car from the lifting gear, which seems more difficult than the first option.

- The rack has to be lifted high (to avoid its legs from touching the ground which may cause the car to drift or the rack to fall off). However, you also need to make sure that it's up there *stable*, because if it is dropped, then we're done.

- More power for the improved lifting mechanism is required for sure.


As mentioned above, due to lack of time, we decided to avoid carrying the racks for now `(V_0.2)`, as this is not the main goal of our project, and dedicate our efforts in more goal-oriented directions.
