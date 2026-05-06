import os, shutil

mapping = {
    "2023-05-14-Controller-Design-Day-00": "0001_Controller_Design_0000",
    "2024-05-15-Controller-Design-Day-01": "0002_Controller_Design_0001",
    "2024-05-20-Controller-Design-Day-02": "0003_Controller_Design_0002",
    "2024-05-24-Controller-Design-Day-03": "0004_Controller_Design_0003",
    "2024-05-28-Controller-Design-Day-04": "0005_Controller_Design_0004",
    "2024-06-03-Controller-Design-Day-05": "0006_Controller_Design_0005",
    "2024-06-20-Controller-Design-Day-06": "0007_Controller_Design_0006",
    "2024-07-09-Controller-Design-Day-07": "0008_Controller_Design_0007",
    "2024-07-29-Controller-Design-Day-08": "0009_Controller_Design_0008",
    "2024-08-07-Controller-Design-Day-09": "0010_Controller_Design_0009",
    "2024-08-12-Paper-Review-01": "0011_Paper_Review_0001",
    "2024-08-26-System-Modeling-01": "0012_System_Modeling_0001",
}

os.makedirs("contents/_articles", exist_ok=True)

for old, new in mapping.items():
    src = f"contents/_posts/{old}"
    dst = f"contents/_articles/{new}"
    if not os.path.isdir(src):
        print(f"SKIP (not found): {src}")
        continue

    shutil.copytree(src, dst, dirs_exist_ok=True)

    md = f"{dst}/index.md"
    if os.path.isfile(md):
        content = open(md, encoding="utf-8").read()

        # Fix thumbnail relative path -> absolute
        content = content.replace(
            "thumbnail: ./assets/img/", f"thumbnail: /articles/{new}/assets/img/"
        )
        content = content.replace(
            "thumbnail: ./assets/video/", f"thumbnail: /articles/{new}/assets/video/"
        )

        # Insert permalink into frontmatter after layout: line
        lines = content.split("\n")
        insert_after = None
        for i, line in enumerate(lines):
            if i > 0 and line.startswith("layout:"):
                insert_after = i
                break
        if insert_after and not any(l.startswith("permalink:") for l in lines):
            lines.insert(insert_after + 1, f"permalink: /articles/{new}/")
            content = "\n".join(lines)

        open(md, "w", encoding="utf-8").write(content)
    print(f"Migrated: {old} -> {new}")

print("Done.")
