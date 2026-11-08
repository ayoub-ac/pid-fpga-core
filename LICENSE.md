# License

This IP core is dual-licensed.

SPDX-License-Identifier expression: `GPL-3.0-or-later OR LicenseRef-Commercial`

## GPL-3.0-or-later (open-source use)

Copyright (C) 2026 Ayoub Achour

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

The full GPL-3.0 text is in [COPYING](COPYING). It is also published by the FSF: <https://www.gnu.org/licenses/gpl-3.0.txt>.

**What this means in practice**: if you use this RTL in a project, the project itself (including any FPGA bitstream produced from it) must also be released under GPL-3.0-or-later. The GPL is "viral" — your downstream users must receive the same rights you received, including the right to the complete corresponding source.

If that is fine for your project, you are good to go. No fee, no email needed.

## Commercial license (closed-source / proprietary use)

If you want to use this core in a product whose source is not GPL — for example a closed-source motor drive, a brushless / BLDC controller IC, an autopilot for drones / UAVs, an industrial PLC firmware, an SMPS / DC-DC voltage regulator with digital control, a process-control product, or any FPGA-based motion / temperature / pressure regulator — you need a commercial license.

The commercial license:

- removes the GPL viral requirement,
- grants you (and your company) the right to integrate the core into closed-source products,
- gives you a clear, written grant you can show to your customers and to a due-diligence auditor.

Commercial licenses are available on request: open a GitHub issue titled 'Commercial license request'. Do not post confidential details in the issue. Any commercial license is provided as is, without warranty, support or maintenance. This core is unverified (see README).

## Trademarks

"PID" is a generic term for the proportional-integral-derivative control law used in industrial control since at least the 1920s; no trademark claim is made on the name. References to specific control textbooks (Astrom & Hagglund, Franklin & Powell) are made for technical clarity only.

## Patents

The discrete-time positional-form PID algorithm and the standard anti-windup techniques (conditional integration / "clamp" and back-calculation per Astrom & Hagglund 1995) are described in publicly accessible engineering literature. The author makes no warranty about patent freedom; if you ship hardware globally you are responsible for your own patent due diligence.
