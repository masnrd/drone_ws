# worlds

Custom world files for simulation.

## Installation
SDF files do not use relative links. Hence, a Python file is used to parse the relative links in the world files into absolute links, and the user is required to move the generated files to the `~/.simulation-gazebo/worlds` directory to set up the custom worlds.

```bash
python3 parse_rel_paths.py
```

Then, move all `.sdf` files in the `out` directory into the simulation directory:
```bash
mv out/* ~/.simulation-gazebo/worlds/
```
