# The panel's fonts

Built into `sally-panel` (`include_bytes!` in `src/ui/theme.rs`):

| file | what | copyright | license |
|---|---|---|---|
| `IBMPlexSansVar-Roman.ttf` | everything read: the variable font, from which the panel cuts its weights (450, 500, 600) and its condensed width | Copyright © 2017 IBM Corp. with Reserved Font Name "Plex" | SIL Open Font License 1.1, `OFL-IBM-Plex.txt` |
| `IBMPlexMono-Medium.ttf` | what a machine named: serials, codenames, parameters | Copyright © 2017 IBM Corp. with Reserved Font Name "Plex" | SIL Open Font License 1.1, `OFL-IBM-Plex.txt` |
| `Phosphor-Bold.ttf`, `Phosphor-Fill.ttf` | the icons | Copyright (c) 2020-2021 Phosphor Icons | MIT, `MIT-Phosphor.txt` |

The Plex files are IBM's own, as its `@ibm/plex` 6.4.1 package has them (unpacked from its WOFF2 files, the font data
as it is). The Phosphor files are `@phosphor-icons/web`'s Bold and Fill fonts cut down to the icons the panel uses:
the ones `src/ui/icons.rs` lists, each by the code point Phosphor's own `selection.json` gives its name (the same in
every weight).

To add an icon: add its constant to `icons.rs` with its code point from `selection.json`, and cut both fonts again
from Phosphor's files with every code point of `icons.rs`:

    pyftsubset Phosphor-Bold.ttf --unicodes=U+e038,U+e058,... --layout-features='' --no-hinting --name-IDs='*' \
        --notdef-outline --output-file=crates/sally-panel/fonts/Phosphor-Bold.ttf

A device's own symbol (a controller, a tracker, a base station as SteamVR's status window shows it) is not here: the
panel reads it from the SteamVR install on the PC (`src/symbols.rs`), and these icons stand in where there is none.
