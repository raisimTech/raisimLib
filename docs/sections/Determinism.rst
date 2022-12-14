#############################
Determinism
#############################

Raisim can be fully deterministic given the contact solver iteration order.
This order can be set by ``raisim::World::setContactSolverIterationOrder``.

Determinism is important because of the two reasons.
First, determinism allows you to control stochasticity of simulation yourself.
This makes many sampling-based stochastic optimal control methods possible.
Second, determinism often means bug-free reliable simulation.
Non-deterministic simulation is often caused by unintended memory read/write.
It is very unlikely that programmers of a simulator intentionally implemented stochasticity in simulation.
There are very few exceptions, such as stochastic contact solvers, but they are very rare.
