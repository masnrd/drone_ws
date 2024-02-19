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

## Adding Custom Worlds from Blender
Assuming you have a properly rotated and scaled model in Blender, export as a `.dae` (Collada) file with:
- "Up" axis set to Z
- "Forward" axis set to X

Do note that since the "forward" axis is set to X (and Blender's default when viewing from the Top is to set Y as forward), you may need to rotate the model where necessary.

Next, copy all dependencies (e.g. texture image files and the `.dae` file itself) to `./worlds/attachments`. Import the model into Gazebo Sim, and save it as a world file (`.sdf`).

Finally, copy necessary tags into the `.sdf` file (refer to `SUTD_field.sdf` as an example).

Finally, run the above installation script and move the `sdf` file accordingly.