"""
Processes .sdf files in this directory, replacing all relative links (surrounded by "${}") with absolute links, as the SDF file format does not support relative links.

Output is written to a './out' directory.
"""

from pathlib import Path

OUT_DIR = Path("./out")

def parseSDF(sdf: Path, new_sdf: Path):
    with sdf.open() as f:
        contents = f.read()
        i = 0
        while "${" in contents:
            # Extract relative link
            start_i = contents.find("${")
            end_i = contents[start_i:].index("}") + start_i + 1
            rel_link = contents[start_i:end_i][2:-1]

            # Resolve to absolute link
            abs_link = str(Path(rel_link).resolve())

            # Replace contents
            contents = contents[:start_i] + abs_link + contents[end_i:]
            i+=1
            
        new_sdf.touch(exist_ok=True)
        with new_sdf.open('w') as newf:
            newf.write(contents)
        

def main():
    OUT_DIR.mkdir(exist_ok=True)
    for p in Path('.').glob("*.sdf"):
        new_p = OUT_DIR.joinpath(p.name)
        parseSDF(p, new_p)
        print(f"Processed: {p} -> {new_p}")

if __name__ == "__main__":
    main()
    
    
