An attempt to improve on the shortcomings of Parallel Printer by implementing a parallel variation of the Axis Aligned Height Field decomposition logic.

# Notes

- Algorithm stagnates suboptimally
- - Select a random path or sth idk?
- - - Prioritise based on certain parameters?


- Reduce the verbosity for CLI readout
    - As a build option...? cli opt...?
    - Remaining blocks + growth iteration combined
    - Block computed score for -> best score op in one


# Outstanding Bugs

## Major

## Moderate

- Unusual out of bounds exception somewhere in code.
    - Randomly occurs. Not easily reproducible.
    - Every time, it's an overflow rather than an underflow.
    - There are mathematical guards against this. So why is it happening?
    - Additional fixes now implemented, seem to work?

## Minor
