# RAGE Plan: Relevance-Aware GEnerative Planning

By Juan Carlos Saborío Morales  
DFKI Niedersachsen Lab  
juan.saborio@dfki.de  

Formerly KBS Group at the Institute of Computer Science, University of Osnabrück.

Summary
--------
This is a relevance-based POMDP planner that generates action preferences internally and estimates "feature" values in order to reduce the dimensionality of otherwise large, complex POMDP's.

In a nutshell:
  * PO-MCTS
  * Goal-driven action selection with PGS rollouts
  * Potential-based reward shaping bias with PGS
  * Feature relevace estimation through Incremental refinement (IRE):
    * feature-aware rollouts
    * feature-aware UCB1
  * Feature (de)activation
  * POMDP test domains (Cellar, Big Foot)

The underlying MCTS sampling is based on POMCP, from the NIPS 2010 paper "Online Monte-Carlo Planning in Large POMDPs" by David Silver and Joel Veness.  This version of RAGE still uses some POMCP code.

As of Summer 2020 this is one of the most efficient POMDP planners in *very* large domains that doesn't rely on explicit rules or prior knowledge.  This program was used (in different versions) to obtain experimental results in the related publications.

Related publications
--------------------

* Saborı́o, J.C. (2019) "Relevance-based Online Planning in Complex POMDPs".  PhD. Thesis, University of Osnabrück.

* Saborı́o, J. C. and Hertzberg, J. (2019). "Efficient planning under uncertainty with incremental refinement."
In Proc. 35th Conference on Uncertainty in Artificial Intelligence, UAI 2019, Tel Aviv, Israel, July 22-25, 2019, page 112

* Saborı́o, J. C. and Hertzberg, J. (2019). "Planning under uncertainty through goal-driven action selection."
In van den Herik, J. and Rocha, A. P., eds, Agents and Artificial Intelligence, pages 182–201, Cham. Springer International
Publishing.

* Saborı́o, J. C. and Hertzberg, J. (2018). "Towards domain-independent biases for action selection in robotic 
task-planning under uncertainty."
In Proc. 10th Intl. Conference on Agents and Artificial Intelligence - Volume 2: ICAART,, pages 85–93. INSTICC, SciTePress.
