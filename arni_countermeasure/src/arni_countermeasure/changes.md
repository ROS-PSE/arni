
##changes in arni_countermeasure:

###Constraint:
 + added attribute name


###constraint-yaml-specification:
changed to:

    constraints:
      constraint_name:
        constraint:
          and:
            not:
              seuid2: {cpu_load: high}
            seuid1: {cpu_load: low, mem_avg: low}
            seuid2: {mem_avg: low}
        min_reaction_interval: 20
        reactions:
          one: {action: stop, autonomy_level: 13, node: nodeid}
          two: {action: publish, autonomy_level: 13, message: some odd message, node: nodeid, loglevel: warn}
    reaction_autonomy_level: 50


advantage now: can be written in the parameter server as it is.
No need to load as text file or something like that.

###ReactionPublishRosoutNode

 + added attribute for loglevel

###Reaction

 + changed autonomy_level to public

 ###helper
 added a helper class to provide 