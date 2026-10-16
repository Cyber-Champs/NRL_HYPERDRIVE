Import("env")
import os
from pathlib import Path

# Libraries whose linkage depends entirely on global-constructor side effects
# (e.g. HexaNRL's example opmodes self-register via REGISTER_OPMODE_EX with no
# other symbol referencing them) -- a plain -l link lets the linker prune those
# .o's out of the archive since nothing resolves an "undefined symbol" against
# them. --whole-archive forces every object in, so the constructors still run.
WHOLE_ARCHIVE_LIBS = {"HexaNRL"}

project_dir = Path(env["PROJECT_DIR"])
lib_search_roots = [project_dir / "lib", project_dir.parent / "lib"]

archives = []
for lib_root in lib_search_roots:
    if not lib_root.is_dir():
        continue
    for a_file in lib_root.glob("*/lib/xtensa-esp32s3/lib*_static.a"):
        archives.append(str(a_file))
        lib_dir_name = a_file.parents[2].name  # .../<LibName>/lib/xtensa-esp32s3/lib*.a
        if lib_dir_name in WHOLE_ARCHIVE_LIBS:
            env.Append(LINKFLAGS=["-Wl,--whole-archive", str(a_file), "-Wl,--no-whole-archive"])
        else:
            lib_name = a_file.stem[len("lib"):]  # "libHexaServos_static" -> "HexaServos_static"
            env.Append(LIBPATH=[str(a_file.parent)])
            env.Append(LIBS=[lib_name])

# Make a changed .a actually force a relink.
#
# The appends above tell the LINKER where the archives are; they tell SCons
# nothing about them being INPUTS. So when only a .a changes, the build is judged
# up to date, the link is skipped, and the upload re-flashes the previous binary
# while reporting complete success.
#
# Not theoretical: replacing an .a and running `-t upload` three times in a row
# re-flashed the same stale firmware every time, each with "Wrote ... bytes" and
# "Hash of data verified". The .a was 13 minutes newer than the firmware.elf
# supposedly built from it. Only `-t clean` broke it.
#
# Why this matters for students: a mid-season update replaces these archives. A
# student updating an EXISTING project -- which they will, since their opmodes
# live there -- would silently keep running the old firmware: no error, no
# warning, a successful-looking upload. They would report that the update did not
# work and it could not be diagnosed remotely.
#
# Declaring the dependency is the only fix that cannot be forgotten. "Run Clean
# after updating" relies on memory in precisely the case where being wrong is
# invisible.
if archives:
    # NOTE the literal "firmware.elf". In a `pre:` script ${PROGNAME} has not been
    # resolved yet -- it substitutes to "program", so
    # "$BUILD_DIR/${PROGNAME}.elf" names .pio/build/<env>/program.elf, a node that
    # is never built. The dependency is accepted in silence and does nothing,
    # which is indistinguishable from not having written it. PlatformIO links
    # $BUILD_DIR/firmware.elf; that is the node to hang this on.
    env.Depends(os.path.join(env.subst("$BUILD_DIR"), "firmware.elf"), archives)
