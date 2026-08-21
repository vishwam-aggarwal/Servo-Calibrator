# Motor types

Inventory numbers used in `study_range.py`'s output filenames
(`type<N>_unit<M>_<timestamp>_*.csv`) for the multi-servo characterization
study. Plain integers only, by design — no product names in filenames,
matching how the study itself doesn't care what a servo is called, only
how it actually behaves.

| Type | Model | Units |
|---|---|---|
| 1 | Miuzei 25kg Servo | 3 (unit1, unit2, unit3) |
| 2 | Knockoff MG996R | 3 (unit1, unit2, unit3) |
| 3 | MG90D | 3 (unit1, unit2, unit3) |

9 units total across 3 types, **study complete** (2026-08-21) — every
unit has a full accuracy run; see `CLAUDE.md`'s dated entry for the
final results table. Originally planned as 8 units (`type1` x2); the
user ordered a third `type1` unit specifically to give every family 3
units. `type0`/`unit0` is reserved by `study_range.py` itself for
unlabeled/test runs (smoke tests, dev runs) when `MOTOR_TYPE`/`UNIT`
aren't passed — never used for real study data.
