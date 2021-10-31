
To run an example:

```rust
cargo run --example clohessy_wiltshire
```

The examples will output a csv file in the run directory with the trajectories of each of the Entities which have a FullState component.

With this data, the reader is encouraged to use third-party tools to generate plots or animations.

An example in python is shown:
```python
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

# read the csv
trajectories_df = pd.read_csv("my_scenario_results.csv")


```

Generate plots of each entity's X,Y,Z position trajectory
```python

entity_plots = {}

# Names are stored as: "Entity#.x" -> #: number, x: state component
entity_names = [x for x in trajectories_df.filter(like='Entity').columns.to_list() if "." not in x]

for name in entity_names:

    # Example of trajectory X,Y,Z states occupying first 3 columns with this name
    x = trajectories_df[name]
    y = trajectories_df[name + ".1"]
    z = trajectories_df[name + ".2"]

    # plot initial state
    # plt.plot(x.iloc[0], y.iloc[0], z.iloc[0], c='k', marker='o')

    # plot time history
    entity_plots[name] = ax.plot(x, y, z, c='b', marker='o')[0]

```

Show the plot
```python
plt.show()
```

Please see the python scripts in some of the included examples for more details.
