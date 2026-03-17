# AirGradient Go Complete On-Device UI Specification

Last updated: 2026-03-16

## 1. Purpose

This document specifies the complete on-device UI for AirGradient Go as currently intended by the interactive simulator and product direction, but adapted for implementation on:

1. `ESP32-C5`
2. `u8g2`
3. Monochrome e-paper display at `122 x 250 px`
4. Three front buttons: `Left`, `Menu`, `Right`

This is a firmware UI specification, not a web specification.

The goal is to bridge the current firmware repository from its partial dashboard-only UI to the full device UI including:

1. Home screen
2. Metric detail views
3. Main menu
4. Tracking control
5. Source tagging
6. Settings and submenus
7. Device lock and auto-lock
8. About Device screen
9. Clear Data confirmation
10. Display Off mode
11. Shutdown screen

## 2. Source Of Truth

The intended UI behavior in this document is derived from two sources.

### 2.1 UX Source Of Truth

Primary handoff copies for firmware work:

1. `./airgradient-go-ui-reference/AirGradientGoDeviceSvg.vue`
2. `./airgradient-go-ui-reference/airgradient-go-simulator.vue`

The copied reference files contain the most complete screen and menu model and are intended to be usable even when the firmware developer does not have access to the website app.

### 2.2 Firmware Baseline

The current firmware repository was inspected here:

1. GitHub: [samuelbles07/airgradient-go](https://github.com/samuelbles07/airgradient-go)
2. Local inspection clone used for this specification: `/tmp/airgradient-go-spec-src`

Key files inspected:

1. `/tmp/airgradient-go-spec-src/main/go.cpp`
2. `/tmp/airgradient-go-spec-src/main/airgradient-go.cpp`
3. `/tmp/airgradient-go-spec-src/main/go_constants.h`
4. `/tmp/airgradient-go-spec-src/main/button_service.h`
5. `/tmp/airgradient-go-spec-src/main/ui/dashboard_ui.h`
6. `/tmp/airgradient-go-spec-src/main/ui/dashboard_ui.cpp`
7. `/tmp/airgradient-go-spec-src/BLE_STREAM.md`

### 2.3 Priority When There Is A Conflict

If there is a conflict, use this priority:

1. This specification
2. The current simulator behavior
3. The existing firmware implementation

Reason:

1. The simulator is the best available representation of the intended complete IA
2. The firmware repo currently implements only part of that IA

## 3. Current Firmware Repo Status

The current external firmware repo already provides these foundations:

1. `ButtonService` with `Press`, `Release`, `ShortPress`, `LongPress`
2. Three touch buttons via CAP1203 plus physical inputs
3. E-paper panel driver and `u8g2`-based dashboard rendering
4. BLE streaming and config writes
5. GPS service
6. NAND storage
7. Device states: `IDLE`, `INACTIVE`, `SYNC`, `TRACKING`, `SHUTDOWN`

What the firmware repo currently does not yet provide as a complete on-device IA:

1. Main menu framework
2. Settings navigation
3. Tracking start/stop from UI menu
4. Source tagging UI
5. Device lock UI
6. Auto-lock behavior
7. Mode setting
8. About Device screen
9. Clear Data confirmation flow
10. Display Off mode
11. Shutdown message screen

### 3.1 Simulator Views Already Partially Represented In Firmware

The current firmware already contains parts of the simulator’s visual structure. This is important because the complete UI should be built by extending and aligning that existing work, not by treating firmware as empty.

Already present in the current firmware UI code:

1. Top toolbar or status-header concept
2. PM2.5 hero label and value area
3. CO2 hero label and value area
4. Divider between hero area and lower metrics
5. Lower metrics grid concept
6. Status icons for connection or device state
7. Partial-refresh rendering infrastructure for dashboard-like updates

Concrete mappings to the simulator:

1. Simulator top status toolbar
   1. Firmware already has a top status region with time and status icons in `main/dashboard/dashboard.cpp`
   2. Firmware already has top-right status icons in `main/ui/dashboard_ui.cpp`
2. Simulator PM2.5 / CO2 hero blocks
   1. Firmware already draws PM2.5 and CO2 labels and large values
3. Simulator lower metric section
   1. Firmware already draws Temp, Humidity, TVOC, NOx, Pressure, and Altitude labels
   2. Firmware already has lower-value drawing rectangles for the same family of metrics
4. Simulator status indicators
   1. Firmware already includes tracking, sync, GPS, charging, battery, and BLE-related display elements depending on codepath

### 3.2 Existing Firmware UI Should Be Reused, Not Discarded

The current firmware implementation can and should continue to exist as the basis for the final device UI.

Recommended interpretation:

1. Reuse the existing dashboard rendering architecture
2. Reuse existing icon bitmaps or status rendering where practical
3. Reuse partial-refresh logic
4. Reuse current PM, CO2, and lower-metric data binding paths

What should change:

1. Exact sizes may need adjustment
2. Exact positions may need adjustment
3. Dividers and spacing may need adjustment
4. Font usage may need adjustment to match the final typography rules in this spec
5. Header composition may need adjustment so it matches the simulator toolbar structure
6. The existing dashboard should be treated as an implementation base, not as the final visual specification

### 3.3 Reuse Versus Alignment Rule

When the existing firmware UI differs from the simulator reference:

1. Keep the existing firmware code structure if it is technically sound
2. Adjust coordinates, spacing, dimensions, and draw order to match this specification
3. Do not preserve an old layout purely because it already exists in code

In short:

1. Reuse the implementation approach
2. Align the visual result to the simulator-derived spec

## 4. Hardware And Platform Constraints

### 4.1 Display

Target display characteristics:

1. Resolution: `122 x 250 px`
2. Monochrome only
3. E-paper refresh cost is meaningful
4. Ghosting mitigation matters
5. Full-screen redraws should be minimized

Implementation implications:

1. Use partial refresh for small dynamic regions when possible
2. Use full refresh when switching between fundamentally different screens
3. Avoid dense decorative graphics
4. Prefer text and simple line icons

### 4.2 Input Model

The user-facing front UI uses three logical buttons:

1. `Left`
2. `Menu`
3. `Right`

Current firmware wiring confirms the three touch controls:

1. CAP1203 `CS1 = Right`
2. CAP1203 `CS2 = Left`
3. CAP1203 `CS3 = Enter`

There are also physical buttons in firmware, but the front-screen UI specified here is built around the three logical front inputs above.

### 4.3 Long Press Duration

The current firmware baseline uses:

1. `GO_BUTTON_LONG_PRESS_MS = 2500`

The current intended UI target is:

1. `2000 ms`

Firmware should update long-press handling to `2 seconds` so it matches the intended lock/unlock behavior.

### 4.4 Reference Design Grid

The simulator uses a logical internal design grid of:

1. `144 x 296`

That logical grid is then scaled into the real display size:

1. Physical display: `122 x 250`
2. X scale: `122 / 144 = 0.847222...`
3. Y scale: `250 / 296 = 0.844595...`

For firmware implementation, use this conversion:

1. `physical_x = round(logical_x * 122 / 144)`
2. `physical_y = round(logical_y * 250 / 296)`
3. `physical_w = round(logical_w * 122 / 144)`
4. `physical_h = round(logical_h * 250 / 296)`

This means the simulator can be treated as the exact layout source, even though it is authored on a `144 x 296` logical canvas.

### 4.5 Layout Fidelity Rule

For firmware, preserve these in order of priority:

1. Block positions and spacing
2. Row heights and divider placement
3. Relative font hierarchy
4. Icon placement and alignment
5. Exact font rendering only as far as `u8g2` allows

Because `u8g2` fonts will not exactly match the browser fonts used in the simulator, firmware should treat the simulator geometry as strict and the typography as approximate within those bounds.

Implementation reference for all coordinates in this document:

1. `apps/shared/feature-documentation/airgradient-go-ui-reference/AirGradientGoDeviceSvg.vue`

That copied file should be treated as the coordinate-level reference snapshot for firmware work.

## 5. Terminology

Use these user-facing terms consistently:

1. `Tracking`
2. `Tag`
3. `Display Interval`
4. `PM Interval`
5. `Other Sensor Int.`
6. `GPS Mode`
7. `Mode`
8. `Auto Lock`
9. `About Device`
10. `Display Off`

Do not use these older alternatives in the UI:

1. `Recording`
2. `Session`
3. `Interval` as a standalone settings label

## 6. Global Device UI Model

The UI has three high-level power phases:

1. `On`
2. `Shutting Down`
3. `Off`

Inside `On`, the screen has these UI modes:

1. `main`
2. `menu`
3. `settings`
4. `settings-choice`
5. `about`
6. `confirm`
7. `tag-list`

These UI modes are not the same as the device runtime states in firmware.

Firmware runtime states remain:

1. `IDLE`
2. `INACTIVE`
3. `SYNC`
4. `TRACKING`
5. `SHUTDOWN`

Recommended mapping:

1. In normal powered-on use, the on-screen UI runs while the device is in `IDLE` or `TRACKING`
2. During `SYNC`, the UI should either remain on the home screen with sync status visible or temporarily limit interactions if required by implementation
3. `SHUTDOWN` should show the shutdown screen before the display is cleared or put to sleep

## 7. Global Interaction Rules

### 7.1 Home Screen

On the home screen:

1. `Menu short press` opens the main menu
2. `Left short press` moves to the previous metric detail view
3. `Right short press` moves to the next metric detail view
4. If `Display Off` is active, `Left` and `Right` do nothing on the main screen
5. If `Display Off` is active, `Menu` must still open the menu

### 7.2 List Screens

For all list-like screens:

1. `Left` moves selection up or previous
2. `Right` moves selection down or next
3. `Menu` activates the selected row

### 7.3 Locking

Device lock behavior:

1. Long press `Menu` for `2 seconds` toggles the lock state
2. When locked, short presses on `Left` and `Right` do nothing and show a snackbar
3. When locked, short press `Menu` does not open a menu and instead shows a snackbar
4. Unlock also uses long press `Menu` for `2 seconds`

Required snackbar text when locked and the user attempts input:

1. `Long press Menu 2s to unlock`

Required snackbar text on unlock:

1. `Buttons unlocked`

Required snackbar text on manual lock:

1. `Buttons locked`

### 7.4 Auto-Lock

Auto-lock requirements:

1. Auto-lock is configurable in settings
2. Default is `10 seconds`
3. If the device is unlocked and there is no user interaction for the configured timeout, it must auto-lock
4. Before auto-locking, the UI must return to the home screen
5. After auto-locking, the snackbar must appear on the home screen

Required snackbar text:

1. `Device auto-locked`

User interaction that resets the timer:

1. Any accepted short press when unlocked
2. Any menu navigation input when unlocked

User interaction that does not reset the timer:

1. Inputs while powered off
2. Inputs blocked by lock

## 8. Screen Inventory

The complete on-device UI contains these screens or screen families:

1. Home dashboard
2. Metric detail views
3. Main menu
4. Tag list
5. Settings
6. Settings choice list
7. About Device
8. Clear Data confirmation
9. Display Off home variant
10. Shutdown screen
11. Powered-off blank state

## 9. Home Dashboard

### 9.1 Purpose

The home dashboard is the primary always-available readout.

### 9.2 Layout Zones

Recommended vertical layout on a `122 x 250` display:

1. Top status bar
2. PM2.5 label and value block
3. CO2 label and value block
4. Divider line
5. Two-column, three-row secondary metrics grid
6. Brand logo area near the bottom
7. Snackbar overlay area at the bottom when needed

### 9.2.1 Exact Vertical Structure

Using the simulator logical grid:

1. Status toolbar occupies logical `y = 0..22`
2. Toolbar bottom divider line at logical `y = 22`
3. PM block at logical `y = 32..84`
4. CO2 block at logical `y = 88..144`
5. Main divider at logical `y = 150`
6. Secondary metrics section starts at logical `y = 158`
7. Bottom logo sits at logical `y = 262..290`
8. Snackbar overlays logical `y = 275..296`

Physical equivalents on `122 x 250`:

1. Toolbar bottom divider at about `y = 19`
2. PM block at about `y = 27..71`
3. CO2 block at about `y = 74..122`
4. Main divider at about `y = 127`
5. Secondary metrics section starts at about `y = 133`
6. Bottom logo at about `y = 221..245`
7. Snackbar at about `y = 232..250`

### 9.3 Home Dashboard Content

Status bar content:

1. Lock icon
2. `BLE` indicator
3. `WiFi` indicator
4. GPS indicator
5. Battery icon

Battery must remain right-aligned.
Battery rendering should follow the current firmware implementation. No product change is requested for battery status behavior or battery icon semantics.

PM hero block:

1. Label: `PM2.5 (µg/m³)` or `PM2.5 (USAQI)` depending on setting
2. Large numeric value

CO2 hero block:

1. Label: `CO2 (ppm)`
2. Large numeric value

Secondary metrics grid:

1. `Temp`
2. `Humidity`
3. `TVOC`
4. `NOx`
5. `Pressure`
6. `Altitude`

Bottom brand area:

1. Show the AirGradient logo on the main view
2. Leave a small bottom padding below the logo

### 9.3.1 Exact Home Layout Geometry

Use these simulator-derived logical rectangles.

Status bar:

1. Divider line: `x=0 y=22 w=144 h=1`
2. Left icon cluster anchor: `translate(4, 2)` with icon scale `1.2`
3. Tracking dot center when active: `cx=116 cy=12`
4. Battery icon group anchor: `translate(117, 2)` with icon scale `1.2`

PM block:

1. Background/selectable area: `x=0 y=32 w=144 h=52`
2. Label baseline: `x=72 y=48`, centered
3. Value baseline: `x=72 y=80`, centered

CO2 block:

1. Background/selectable area: `x=0 y=88 w=144 h=56`
2. Label baseline: `x=72 y=106`, centered
3. Value baseline: `x=72 y=139`, centered

Main divider:

1. `x=0 y=150 w=144 h=2`

Secondary section:

1. Vertical divider: `x=72 y=158 w=1 h=102`
2. Horizontal divider row 1 -> row 2: `x=0 y=192 w=144 h=1`
3. Horizontal divider row 2 -> row 3 or chart: `x=0 y=225 or 226`
4. Bottom line above logo area: `x=0 y=260 w=144 h=1`

Bottom logo:

1. `x=0 y=262 w=144 h=28`

Snackbar:

1. Background: `x=0 y=275 w=144 h=21`
2. Snackbar text center: `x=72 y=285.5`

### 9.3.2 Physical Pixel Equivalents

For the main blocks, use these physical approximations after scaling:

1. PM block: `x=0 y=27 w=122 h=44`
2. PM label baseline: centered at `y=41`
3. PM value baseline: centered at `y=68`
4. CO2 block: `x=0 y=74 w=122 h=47`
5. CO2 label baseline: centered at `y=90`
6. CO2 value baseline: centered at `y=117`
7. Main divider: `x=0 y=127 w=122 h=2`
8. Secondary section top: about `y=133`
9. Bottom logo: `x=0 y=221 w=122 h=24`
10. Snackbar: `x=0 y=232 w=122 h=18`

### 9.3.3 Typography Hierarchy

The simulator now uses exactly four logical text sizes:

1. `34 px`
2. `14 px`
3. `12 px`
4. `10 px`

Role mapping:

1. `34 px`: PM and CO2 hero values
2. `14 px`: PM and CO2 labels, lower metric values, shutdown headline
3. `12 px`: menu rows and About Device title
4. `10 px`: snackbar text, secondary metric labels, About metadata, shutdown subtext, and `BLE`

Recommended firmware fonts:

1. `u8g2_font_6x10_tr`
2. `u8g2_font_10x20_tn`

Font usage mapping:

1. Use `u8g2_font_10x20_tn` for PM and CO2 large values
2. Use `u8g2_font_6x10_tr` for PM and CO2 labels
3. Use `u8g2_font_6x10_tr` for all secondary metric labels
4. Use `u8g2_font_6x10_tr` for all secondary metric values
5. Use `u8g2_font_6x10_tr` for menu rows and submenu rows
6. Use `u8g2_font_6x10_tr` for About Device metadata
7. Use `u8g2_font_6x10_tr` for snackbar text
8. Use `u8g2_font_6x10_tr` for status text such as `BLE` if rendered as text

Implementation note:

1. These two fonts are already used in the active minimal firmware UI path in `main/ui/dashboard_ui.cpp`
2. Because `u8g2_font_10x20_tn` is smaller than the simulator’s browser-rendered hero numerals, preserve hierarchy through spacing and block placement rather than trying to match visual weight exactly

### 9.3.4 Inversion Rules On The Home Screen

When a hero metric is selected:

1. The selected PM or CO2 block gets a dark fill
2. The selected block’s label and value switch to white

When a lower metric is selected:

1. Only that lower cell gets the dark fill
2. The rest of the home layout remains unchanged

When PM is selected:

1. CO2 block gets a faint dark tint behind it in the simulator
2. This is optional in firmware, but preferred for parity

### 9.4 Home Dashboard Selection Model

The default home screen is the `none` selection state.

Supported metric selection cycle:

1. `none`
2. `pm25`
3. `co2`
4. `temp`
5. `humidity`
6. `tvoc`
7. `nox`

Current simulator behavior does not cycle to dedicated detail views for:

1. `pressure`
2. `altitude`

Firmware should keep that behavior unless explicitly extended later.

### 9.4.1 Exact Metric Browsing Order

The metric browse order is exactly:

1. `none`
2. `pm25`
3. `co2`
4. `temp`
5. `humidity`
6. `tvoc`
7. `nox`

Behavior:

1. `Right` advances one step in this order
2. `Left` moves one step backward in this order
3. The browse order wraps at both ends
4. From `none`, pressing `Left` moves to `nox`
5. From `nox`, pressing `Right` moves to `none`

### 9.4.2 What The `none` State Means

The `none` state is the default dashboard state.

In the `none` state:

1. PM and CO2 are shown normally with no hero inversion
2. No lower metric cell is highlighted
3. No chart is shown
4. The AirGradient logo is shown at the bottom
5. The third row shows `Pressure` and `Altitude`, not `Min` and `Max`

### 9.4.3 What A Selected Metric Means

A selected metric is not a separate full-screen page. It is a dashboard variant with one active parameter.

When a metric is selected:

1. That metric becomes visually highlighted
2. The bottom logo area is replaced by a small trend chart
3. The third row changes from `Pressure` and `Altitude` to `Min` and `Max`
4. The rest of the dashboard layout remains in place

This means the simulator behavior is:

1. Dashboard-first
2. Not a full-page drilldown
3. Not a tab strip
4. Not a carousel of completely different screens

### 9.4.4 Selection Persistence Rules

If the user opens the main menu while a metric is selected:

1. The selected metric remains selected behind the menu
2. Exiting the menu returns to that same selected metric state

The selection resets to `none` only when one of these happens:

1. The user explicitly browses back to `none`
2. `Display Off` is selected
3. Auto-lock returns the UI to the home screen
4. Power-on or shutdown flow resets to the default home screen

### 9.5 Secondary Metrics Grid Exact Design

The grid is two columns by three rows.

Logical column structure:

1. Left column content area begins at `x=12`
2. Column divider at `x=72`
3. Right column content area begins at `x=80`

Logical row rectangles:

1. Row 1 left cell: `x=1 y=159 w=70 h=32`
2. Row 1 right cell: `x=73 y=159 w=70 h=32`
3. Row 2 left cell: `x=1 y=193 w=70 h=32`
4. Row 2 right cell: `x=73 y=193 w=70 h=32`
5. Row 3 left cell: `x=1 y=227 w=70 h=32`
6. Row 3 right cell: `x=73 y=227 w=70 h=32`

Logical label/value baselines:

1. Row 1 left label `Temp`: `x=12 y=168`
2. Row 1 left value: `x=12 y=184`
3. Row 1 right label `Humidity`: `x=80 y=168`
4. Row 1 right value: `x=80 y=184`
5. Row 2 left label `TVOC`: `x=12 y=202`
6. Row 2 left value: `x=12 y=218`
7. Row 2 right label `NOx`: `x=80 y=202`
8. Row 2 right value: `x=80 y=218`
9. Row 3 left label `Pressure` or `Min`: `x=12 y=236`
10. Row 3 left value: `x=12 y=252`
11. Row 3 right label `Altitude` or `Max`: `x=80 y=236`
12. Row 3 right value: `x=80 y=252`

Physical equivalents are approximately:

1. Left label/value anchors at `x=10`
2. Right label/value anchors at `x=68`
3. Row 1 value baseline at `y=155`
4. Row 2 value baseline at `y=184`
5. Row 3 value baseline at `y=213`

## 10. Metric Detail Views

Metric detail views are reached from the home screen using `Left` and `Right`.

Each metric detail view should show:

1. The currently selected metric highlighted
2. A compact trend chart in the lower display area
3. Min and max labels for the chart if available

Detail views are required for:

1. PM2.5
2. CO2
3. Temperature
4. Humidity
5. TVOC
6. NOx

### 10.1 Detail View Behavior

1. `Left` moves to previous metric in the cycle
2. `Right` moves to next metric in the cycle
3. `Menu` opens the main menu
4. If `Display Off` is active, detail views must not be shown
5. Entering `Display Off` resets the selected metric back to `none`

### 10.1.1 Which Metrics Produce A Chart

The small chart is shown only when the selected metric is one of:

1. `pm25`
2. `co2`
3. `temp`
4. `humidity`
5. `tvoc`
6. `nox`

The chart is not shown for:

1. `none`
2. `pressure`
3. `altitude`

Pressure and altitude remain static lower-grid metrics in the current intended UI.

### 10.1.2 Highlight Rules Per Metric

If selected metric is `pm25`:

1. PM block is inverted
2. CO2 block gets a faint shaded background
3. Lower grid remains unselected

If selected metric is `co2`:

1. CO2 block is inverted
2. PM block remains normal
3. Lower grid remains unselected

If selected metric is `temp`, `humidity`, `tvoc`, or `nox`:

1. Only that corresponding lower-grid cell is inverted
2. PM and CO2 remain normal
3. Other lower-grid cells remain normal

### 10.1.3 Lower Grid Substitution While Chart Is Visible

When a chart is visible:

1. Row 1 still shows `Temp` and `Humidity`
2. Row 2 still shows `TVOC` and `NOx`
3. Row 3 no longer shows `Pressure` and `Altitude`
4. Row 3 left becomes `Min`
5. Row 3 right becomes `Max`

This substitution is required because the chart occupies the bottom brand area and the selected metric needs contextual trend information.

### 10.2 Chart Layout

When a detail metric is selected, the bottom logo area is replaced by a compact chart.

Simulator chart rectangle:

1. Logical chart box: `x=0 y=262 w=144 h=34`
2. Internal chart padding: `padX=4 padY=4`
3. Effective plot area:
4. `x=4`
5. `y=266`
6. `w=136`
7. `h=26`

Axes:

1. Left axis from `(4, 266)` to `(4, 292)`
2. Bottom axis from `(4, 292)` to `(140, 292)`

Physical equivalents:

1. Chart box about `x=0 y=221 w=122 h=29`
2. Plot area about `x=3 y=225 w=115 h=22`

Min and max cards replace the third row while the chart is visible:

1. Left card label becomes `Min`
2. Right card label becomes `Max`
3. Their geometry stays exactly where `Pressure` and `Altitude` normally sit

### 10.3 Chart Rendering Rules

The small chart should be rendered as a simple monochrome line chart.

Required visual characteristics:

1. Single polyline only
2. No point markers
3. No fill area
4. One left axis line
5. One bottom axis line
6. No tick labels inside the chart area
7. `Min` and `Max` values are shown in the third row instead of inside the plot

Required behavioral characteristics:

1. The chart must update to match the currently selected metric
2. Switching selected metrics must redraw the chart
3. Returning to `none` removes the chart and restores the bottom logo
4. Opening and closing the menu must not clear the current chart selection

### 10.4 Data Requirements For The Small Chart

The current simulator uses placeholder generated data, but firmware should use real historical samples when available.

Recommended firmware behavior:

1. Keep a short rolling buffer per chartable metric
2. Use the most recent `100` samples
3. Normalize the visible values to the plot height
4. Compute visible `Min` and `Max` from the plotted samples

Interpretation:

1. `100` samples is the current product target
2. If the rendered chart has fewer horizontal pixels than source samples, firmware may downsample visually
3. The source window should still be the latest `100` points

If insufficient historical data exists:

1. Plot whatever is available
2. If only one value exists, show a flat line or a short centered line
3. If no values exist, the chart area may remain empty but the axes should still render

### 10.5 Firmware Implementation Interpretation

The small chart is a dashboard enhancement, not a standalone chart screen.

Implementation consequence:

1. Keep the existing dashboard renderer as the base
2. Add a selected-metric state
3. Add a chart subregion renderer
4. Add `Min` and `Max` substitution logic for row 3
5. Do not build a completely separate screen stack just for the charted parameter views

## 11. Main Menu

### 11.1 Menu Items

Main menu items, in order:

1. `Exit Menu`
2. `Start Tracking` or `Stop Tracking`
3. `Add Tag`
4. `Settings`
5. `About Device`

### 11.2 Main Menu Behavior

1. Default selection is `Exit Menu`
2. `Left` and `Right` wrap through items
3. `Menu` activates the selected item

Activation behavior:

1. `Exit Menu` returns to the home screen
2. `Start Tracking` starts tracking, closes the menu, and shows snackbar `Tracking started`
3. `Stop Tracking` stops tracking, closes the menu, and shows snackbar `Tracking stopped`
4. `Add Tag` opens the tag list only when tracking is active
5. `Settings` opens the settings screen
6. `About Device` opens the about screen

When tracking is not active:

1. `Add Tag` must remain visible
2. `Add Tag` must appear greyed out
3. `Add Tag` must not be selectable
4. Left/right menu navigation must skip over it

### 11.3 Main Menu Overlay Layout

The menu and all submenu list screens share one overlay geometry.

Overlay background:

1. Logical rect: `x=0 y=152 w=144 h=144`
2. Physical rect: about `x=0 y=128 w=122 h=122`

Each row:

1. Logical row rect: `x=6 y=(156 + index * 26) w=132 h=24`
2. Physical row rect: about `x=5 y=(132 + index * 22) w=112 h=20`
3. Row corner radius in simulator: `rx=3`

Each row label:

1. Logical text anchor: `x=12 y=(168 + index * 26)`
2. Physical text anchor: about `x=10 y=(142 + index * 22)`

Visual rules:

1. Selected row has dark background
2. Selected row text is light
3. Unselected rows have no fill
4. Unselected row text is dark gray
5. Rows are left aligned, not centered

Navigation layering rule:

1. The first-level main menu keeps the current lower-overlay presentation
2. Second-level screens opened from the main menu, such as `Settings`, `Settings choice`, `Tag list`, `About Device`, and `Clear Data` confirmation, must switch to a full-screen view below the icon bar
3. In those second-level screens, the icon bar remains visible
4. The home content below the icon bar is fully covered by the second-level screen
5. `Settings`, `Tag list`, and `About Device` draw a 1 px horizontal separator directly below the `Back` row to visually separate top navigation chrome from the content area

## 12. Tracking

### 12.1 User-Facing Term

Always use `Tracking`.

### 12.2 Tracking Start And Stop

The user controls tracking from the main menu only.

Behavior:

1. From `IDLE`, selecting `Start Tracking` starts a new tracking run
2. From `TRACKING`, selecting `Stop Tracking` stops tracking
3. Menu label must reflect current state
4. Tracking status icon should appear in the status bar when active

### 12.3 Firmware Integration

The current BLE and runtime model already supports start/stop tracking.

Implementation should connect the menu action to the same tracking state transitions used elsewhere in firmware:

1. Start tracking only from `IDLE`
2. Stop tracking only from `TRACKING`

## 13. Source Tagging

### 13.1 Tag Category Model

The current intended UI uses a single tag list rather than multiple categories.

The menu label remains:

1. `Add Tag`

### 13.2 Tag List

Tag list items, in order:

1. `Exit`
2. `Back`
3. `Traffic Emissions`
4. `Road Dust`
5. `Construction Work`
6. `Biomass Burning`
7. `Garbage Burning`
8. `Factory Emissions`
9. `Smoking/Vaping`
10. `Cooking`
11. `Paint/Solvents`
12. `Other Pollution`

### 13.3 Tag List Behavior

1. Default selection is `Back`
2. `Add Tag` is only available while tracking is active
3. When not tracking, the main-menu `Add Tag` row stays visible but is rendered disabled and cannot be selected
4. The tag screen keeps `Exit` and `Back` fixed at the top
5. Seven tag rows are visible at a time below the fixed top rows
6. Tag navigation is page-based, not smooth-scrolling
7. Within a page, `Left` and `Right` move one row at a time
8. Moving beyond the last visible tag on a page jumps to the next page and selects its first tag
9. Moving above the first visible tag on a page jumps to the previous page and selects its last tag
10. The tag list clamps at the first and last page instead of wrapping
11. Moving up from the first tag on the first page returns selection to `Back`
12. A separator line is drawn below `Back`
13. Selecting a tag closes the menu stack and returns to the main screen

Required snackbar text:

1. `Tag '<tag>' saved`

Recommended firmware storage model:

1. Attach the selected tag to the current tracking route if tracking is active
2. The UI should not expose tag selection when tracking is inactive

## 14. Settings

### 14.1 Settings Items

Settings items, in order:

1. `Exit`
2. `Back`
3. `Units: °C` or `Units: °F`
4. `PM Display: µg/m³` or `PM Display: USAQI`
5. `Display Interval: <value>`
6. `PM Interval: <value>`
7. `Other Sensor Int.: <value>`
8. `GPS Mode: <value>`
9. `Mode: <value>`
10. `Auto Lock: <value>`
11. `Data: Clear Data`

### 14.2 Settings Navigation

1. Default selection is `Back`
2. `Exit` and `Back` stay fixed at the top of the screen
3. Seven setting rows are visible at a time below the fixed top rows
4. Settings navigation is page-based, not smooth-scrolling
5. Within a page, `Left` and `Right` move one row at a time
6. Moving beyond the last visible row on a page jumps to the next page and selects its first row
7. Moving above the first visible row on a page jumps to the previous page and selects its last row
8. The settings list clamps at the first and last page instead of wrapping
9. Moving up from the first setting on the first page returns selection to `Back`
10. A separator line is drawn below `Back`
11. Selecting a configurable setting opens the generic settings-choice list
12. Selecting `Data: Clear Data` opens the confirmation screen

## 15. Settings Choice Screen

This is the generic list used to edit one setting at a time.

### 15.1 Shared Rules

1. Top rows are `Exit` and `Back`
2. Default selection is `Back`
3. `Left` and `Right` wrap through all rows
4. Seven options are visible at a time below the fixed top rows
5. Choosing an option applies it immediately and returns to the parent screen

### 15.2 Settings And Allowed Options

`Units`

1. `°C`
2. `°F`

`PM Display`

1. `µg/m³`
2. `USAQI`

`Display Interval`

1. `1s`
2. `10s`
3. `30s`
4. `60s`
5. `5m`
6. `15m`
7. `1h`
8. `Display Off`

`PM Interval`

1. `1s`
2. `10s`
3. `30s`
4. `60s`
5. `5m`
6. `15m`
7. `1h`
8. `Off`

`Other Sensor Int.`

1. `1s`
2. `10s`
3. `30s`
4. `60s`
5. `5m`
6. `15m`
7. `1h`
8. `Off`

`GPS Mode`

1. `Always Off`
2. `On When Tracking`
3. `Always On`

`Mode`

1. `Stationary`
2. `Portable`
3. `Offline / Airplane Mode`

`Auto Lock`

1. `Off`
2. `10 Seconds`
3. `30 Seconds`
4. `60 Seconds`

### 15.3 Internal Mapping

Recommended internal values for `GPS Mode`:

1. `off`
2. `tracking`
3. `always`

Recommended internal values for auto lock:

1. `0`
2. `10`
3. `30`
4. `60`

### 15.4 Shared List-Screen Design

The following screens use the same row geometry and should look visually identical except for the row labels:

1. Main menu
2. Settings
3. Settings choice
4. Confirm
5. Tag list

The `About Device` screen uses the same row geometry for its interactive top rows, then adds static information below.

Implementation rule:

1. Build one generic list renderer for these screens
2. Reuse identical row spacing, highlight style, and left text padding

## 16. Mode Setting

`Mode` replaces the previous connectivity submenu.

It is a single top-level setting with these choices:

1. `Stationary`
2. `Portable`
3. `Offline / Airplane Mode`

### 16.1 Mode Behavior

`Stationary`

1. `BLE` is off
2. `WiFi` is on
3. Device is expected to use WiFi Local Server/API behavior
4. Internal airplane-mode state is off

`Portable`

1. `BLE` is on
2. `WiFi` is off
3. Device is expected to use BLE Stream behavior
4. Internal airplane-mode state is off

`Offline / Airplane Mode`

1. `BLE` is off
2. `WiFi` is off
3. Internal airplane-mode state is on
4. No WiFi local server and no BLE stream should be exposed in this mode

### 16.2 UI Behavior

1. `Mode` is edited through the generic settings-choice list
2. There is no separate connectivity submenu
3. Bluetooth and WiFi icons in the top bar must reflect the selected mode
4. The selected mode label in settings must stay on one line

## 17. GPS Mode

GPS remains a separate top-level setting and is not part of `Mode`.

GPS mode labels:

1. `Always Off`
2. `On When Tracking`
3. `Always On`

Recommended behavior:

1. `Always Off` disables GPS hardware unless another subsystem explicitly needs it
2. `On When Tracking` enables GPS only while tracking is active
3. `Always On` keeps GPS running whenever the device is powered on

Status icon behavior:

1. GPS icon should show enabled or fix state using the existing firmware conventions

## 18. Display Interval And Display Off

### 18.1 Display Off Label

For `Display Interval`, the last option must be:

1. `Display Off`

Do not use `Off` for this setting.

### 18.2 Display Off Behavior

When `Display Off` is selected:

1. The main display must stop showing all air quality values
2. The display must show only the AirGradient logo
3. `Menu` must still work
4. `Left` and `Right` must do nothing on the main screen
5. Any active metric detail view must be reset back to `none`

Recommended implementation:

1. The display-off home variant should keep the status bar visible
2. The display body below the status bar should primarily read as a logo-only screen
3. The status bar must remain visually secondary to the logo-only state

### 18.3 Exact Display-Off Layout

In the simulator, display-off mode keeps the menu system available and shows the logo centered above the menu-overlay region.

Display-off logo:

1. Logical rect: `x=0 y=134 w=144 h=28`
2. Physical rect: about `x=0 y=113 w=122 h=24`

If the menu is opened while display-off is active:

1. The overlay still begins at logical `y=152`
2. The overlay geometry is identical to normal menu mode
3. The logo remains visible above the overlay

## 19. About Device

### 19.1 Screen Purpose

This is a simple device-information screen, not a marketing page.

### 19.2 Rows

Interactive rows:

1. `Exit`
2. `Back`

Static information content:

1. `AirGradient Go`
2. `Firmware v<current>`
3. `Serial <12-digit uppercase hex>`
4. `Open Source Hardware`

### 19.3 Behavior

1. Default selection is `Back`
2. `Left` and `Right` wrap between `Exit` and `Back`
3. Selecting `Exit` returns to the home screen
4. Selecting `Back` returns to the main menu with `About Device` selected
5. A separator line is drawn below `Back`

The serial number format should be:

1. `12-digit uppercase hex`

Dynamic content rule:

1. Firmware version must be read dynamically from the current device build or runtime metadata
2. Serial must be read dynamically from the real device identity
3. Simulator values are placeholders, not fixed production content

### 19.4 Exact About Layout

Interactive rows:

1. Row 0 `Exit`: logical `x=6 y=30 w=132 h=24`
2. Row 1 `Back`: logical `x=6 y=56 w=132 h=24`
3. Separator below `Back`: logical `x=8 y=82 w=128 h=1`

Static content baselines:

1. Title `AirGradient Go`: `x=12 y=108`
2. Firmware line: `x=12 y=126`
3. Serial line: `x=12 y=142`
4. Open hardware line: `x=12 y=158`

Physical baseline equivalents:

1. Row 0 `Exit`: about `x=5 y=25 w=112 h=20`
2. Row 1 `Back`: about `x=5 y=47 w=112 h=20`
3. Separator below `Back`: about `x=7 y=69 w=108 h=1`
4. Title at about `x=10 y=91`
5. Firmware at about `x=10 y=106`
6. Serial at about `x=10 y=120`
7. Open hardware at about `x=10 y=133`

Style rules:

1. Title uses the menu-row font size and weight
2. Metadata lines use a smaller and lighter font
3. All static text is left aligned

## 20. Clear Data Confirmation

### 20.1 Rows

Confirmation screen rows, in order:

1. `Exit`
2. `Back`
3. `Clear Data?`
4. `No`
5. `Yes`

### 20.2 Behavior

1. Default selection is `Back`
2. `Left` and `Right` wrap through rows
3. `Clear Data?` is informational and should not trigger an action when selected
4. `No` returns to settings
5. `Yes` clears stored data and returns to the home screen

When data is cleared:

1. Tracking should be stopped
2. Last selected tag should be cleared from UI state
3. Snackbar text must be `Data cleared`

Firmware integration:

1. `Clear Data` means erasing the whole flash chip used for stored device data
2. Existing BLE `flashErase` semantics suggest clear is valid only from `IDLE`
3. If the device is currently tracking, stop tracking before clearing or block the action until safe
4. Any route, tag, and on-device history stored on flash must be treated as deleted after this operation

## 21. Snackbar Messages

Snackbar messages should use sentence-style capitalization, not Title Case.

Required messages:

1. `Long press Menu 2s to unlock`
2. `Buttons unlocked`
3. `Buttons locked`
4. `Device auto-locked`
5. `Tracking started`
6. `Tracking stopped`
7. `Data cleared`
8. `Tag '<tag>' saved`

Display rules:

1. Snackbar appears at the bottom of the screen
2. Snackbar overlays the current screen rather than navigating away
3. Message duration target: `3 seconds`
4. Snackbar text uses the standard `10 px` small-text tier

Exact snackbar geometry:

1. Logical background rect: `x=0 y=275 w=144 h=21`
2. Logical text anchor: `x=72 y=285.5`
3. Physical background rect: about `x=0 y=232 w=122 h=18`
4. Physical text center: about `x=61 y=241`

## 22. Status Bar Icons

### 22.1 Required Icons

1. Lock
2. Bluetooth
3. WiFi
4. GPS
5. Battery

### 22.2 Alignment Rules

1. Battery stays right-aligned
2. The other icons form a left-side cluster
3. Icons must not overlap hero labels
4. Time is not shown in the status bar

### 22.2.1 Exact Status Bar Placement

The simulator status bar is composed like this:

1. Left cluster starts at logical `x=4 y=2`
2. Left cluster is drawn at `1.2x` icon scale
3. Lock icon occupies the first slot
4. `BLE` text starts at logical `x=12`
5. WiFi icon group starts at logical `x=32`
6. GPS icon group starts at logical `x=45`
7. Optional tracking dot is near logical `x=116 y=12`
8. Battery group starts at logical `x=117 y=2`

Physical equivalents:

1. Left cluster starts near `x=3 y=2`
2. `BLE` starts near `x=10`
3. WiFi starts near `x=27`
4. GPS starts near `x=38`
5. Battery starts near `x=99`

Implementation note:

1. The battery must remain independently right-anchored
2. Do not treat all status items as one centered row

### 22.3 Icon States

Lock icon:

1. Visible when locking feature exists
2. Filled or emphasized when locked
3. Dimmed when unlocked

Bluetooth icon:

1. Reflects `Bluetooth` setting

WiFi icon:

1. Reflects `WiFi` setting

GPS icon:

1. Use the current firmware implementation and semantics
2. No product change is requested for GPS icon behavior in this phase

Battery icon:

1. Remains visible in all powered-on main screens
2. Should support charging overlay if charging is known
3. Use the current firmware implementation and semantics

## 23. Shutdown Flow

### 23.1 Shutdown Screen

When powering down, show a transient shutdown screen before the display turns off.

Required text:

1. `Powering off...`
2. `See you soon`

The AirGradient logo must appear on the shutdown screen.

### 23.2 Shutdown Behavior

Before displaying shutdown:

1. Return to the home screen
2. Close all menus
3. Clear transient overlays if needed

After the shutdown message:

1. Put the display into its off state
2. Enter the firmware shutdown or deep-sleep path

### 23.3 Powered-Off Screen

The powered-off state should be blank.

### 23.4 Exact Shutdown Layout

Shutdown text positions in the simulator:

1. `Powering off...` centered at logical `x=72 y=136`
2. `See you soon` centered at logical `x=72 y=160`
3. AirGradient logo at logical `x=0 y=258 w=144 h=28`

Physical equivalents:

1. `Powering off...` centered at about `x=61 y=115`
2. `See you soon` centered at about `x=61 y=135`
3. Logo at about `x=0 y=218 w=122 h=24`

Style rules:

1. Primary shutdown line uses medium-bold centered text
2. Secondary line uses smaller, lighter centered text
3. Logo stays near the bottom edge with a little bottom breathing room

## 24. Navigation Details By Screen

### 24.1 Main Screen

1. `Menu` opens main menu
2. `Left` previous metric
3. `Right` next metric
4. If locked, all short presses show lock guidance
5. If display off, `Left` and `Right` do nothing

### 24.2 Menu Screen

1. `Left` previous row, wraps
2. `Right` next row, wraps
3. `Menu` activates selection

### 24.3 Settings Screen

1. `Left` previous row, wraps
2. `Right` next row, wraps
3. `Menu` activates selection

### 24.4 Settings Choice Screen

1. `Left` previous row, wraps
2. `Right` next row, wraps
3. `Menu` applies the selected choice

### 24.5 About Screen

1. `Left` previous row, wraps
2. `Right` next row, wraps
3. `Menu` activates selection

### 24.6 Confirm Screen

1. `Left` previous row, wraps
2. `Right` next row, wraps
3. `Menu` activates selection

### 24.7 Tag List Screen

1. `Left` moves upward and clamps at top
2. `Right` moves downward and clamps at bottom
3. `Menu` activates selection

## 25. Persistent Settings Model

Recommended persisted settings struct:

```c
typedef enum {
  UNITS_C = 0,
  UNITS_F = 1
} UnitsMode;

typedef enum {
  PM_DISPLAY_UGM3 = 0,
  PM_DISPLAY_USAQI = 1
} PMDisplayMode;

typedef enum {
  INTERVAL_1S,
  INTERVAL_10S,
  INTERVAL_30S,
  INTERVAL_60S,
  INTERVAL_5M,
  INTERVAL_15M,
  INTERVAL_1H,
  INTERVAL_OFF
} SensorInterval;

typedef enum {
  DISPLAY_INTERVAL_1S,
  DISPLAY_INTERVAL_10S,
  DISPLAY_INTERVAL_30S,
  DISPLAY_INTERVAL_60S,
  DISPLAY_INTERVAL_5M,
  DISPLAY_INTERVAL_15M,
  DISPLAY_INTERVAL_1H,
  DISPLAY_INTERVAL_OFF
} DisplayInterval;

typedef enum {
  GPS_MODE_OFF = 0,
  GPS_MODE_TRACKING = 1,
  GPS_MODE_ALWAYS = 2
} GPSMode;

typedef enum {
  TOGGLE_OFF = 0,
  TOGGLE_ON = 1
} ToggleState;

typedef enum {
  AUTO_LOCK_OFF = 0,
  AUTO_LOCK_10S = 10,
  AUTO_LOCK_30S = 30,
  AUTO_LOCK_60S = 60
} AutoLockSeconds;

typedef struct {
  UnitsMode units;
  PMDisplayMode pm_display;
  DisplayInterval display_interval;
  SensorInterval pm_interval;
  SensorInterval other_sensor_interval;
  GPSMode gps_mode;
  ToggleState wifi;
  ToggleState bluetooth;
  ToggleState airplane_mode;
  AutoLockSeconds auto_lock_seconds;
} GoUISettings;
```

Recommended defaults:

1. `units = °C`
2. `pm_display = µg/m³`
3. `display_interval = 10s`
4. `pm_interval = 10s`
5. `other_sensor_interval = 10s`
6. `gps_mode = tracking`
7. `wifi = on`
8. `bluetooth = on`
9. `airplane_mode = off`
10. `auto_lock_seconds = 10`

## 26. Screen Rendering Recommendations For u8g2

### 26.1 General

1. Pre-render static basemaps for major screens where possible
2. Keep dynamic regions isolated for partial refresh
3. Use a small set of font sizes only
4. Use uppercase sparingly; most menu items should be Title Case
5. Snackbar text should use sentence case

### 26.2 Screen Families Suitable For Shared Basemaps

Recommended shared basemaps:

1. Home dashboard
2. Metric detail screen
3. Generic list screen with highlight row
4. About Device static text body
5. Shutdown screen

### 26.3 Font Hierarchy

Recommended text sizes:

1. Status labels and small metadata: small font
2. Menu rows and secondary values: medium font
3. PM and CO2 values: large numeric font
4. Snackbar: medium or reduced medium font

### 26.4 Icons

Icons should be simple 1-bit bitmaps:

1. Lock
2. Bluetooth
3. WiFi
4. GPS
5. Battery
6. Tracking
7. Charging if needed

## 27. Recommended Firmware Architecture

Recommended split:

1. `GoUIController`
2. `GoUIRenderer`
3. `GoUISettingsStore`
4. `GoUISnackbar`
5. `GoUIState`

Recommended responsibilities:

`GoUIController`

1. Handle button events
2. Update UI mode
3. Apply menu actions
4. Coordinate lock and auto-lock timers

`GoUIRenderer`

1. Draw the active screen
2. Draw overlays
3. Manage full versus partial refresh decisions

`GoUISettingsStore`

1. Load defaults
2. Persist settings
3. Apply normalization rules such as airplane mode constraints

`GoUISnackbar`

1. Queue one active transient message
2. Expire after timeout

`GoUIState`

1. Hold mode, selection indexes, lock state, tracking state, and power phase

## 28. Recommended Runtime State Additions

The existing runtime state machine can remain mostly unchanged, but UI state should add:

1. `ui_mode`
2. `active_metric`
3. `device_locked`
4. `show_feedback`
5. `feedback_message`
6. `menu_index`
7. `settings_index`
8. `settings_choice_index`
9. `about_index`
10. `confirm_index`
11. `tag_index`
12. `display_off_active`
13. `power_phase`

## 29. Implementation Phasing

Recommended implementation order:

### Phase 1

1. Refactor current dashboard into a reusable home screen renderer
2. Add main menu framework
3. Add settings framework and generic choice list

### Phase 2

1. Add tracking control
2. Add tag list
3. Add about screen
4. Add clear data confirmation

### Phase 3

1. Add lock and auto-lock
2. Add display-off mode
3. Add shutdown screen
4. Add mode-setting behavior

### Phase 4

1. Tune partial refresh strategy
2. Tune iconography and spacing for the real `122 x 250` panel
3. Validate with real button timings and sleep transitions

## 30. Acceptance Criteria

The UI implementation is complete when all conditions below are true.

### 30.1 Navigation

1. Every screen in this specification is reachable from the device itself
2. Every screen has a defined exit path
3. Menu navigation behaves consistently with this document

### 30.2 Settings

1. Every settings item can be viewed and changed on-device
2. Settings persist across reboot
3. Airplane mode rules are enforced

### 30.3 Locking

1. Long press `Menu` for `2s` locks and unlocks the device
2. Auto-lock works at `10s`, `30s`, and `60s`
3. Auto-lock returns to home before locking
4. Locked inputs show the correct snackbar

### 30.4 Display Off

1. `Display Off` shows only the AirGradient logo
2. `Menu` still works while display is off
3. No metric values are shown in display-off home mode

### 30.5 Tracking And Tags

1. Tracking can be started and stopped from the main menu
2. The correct tracking snackbar appears
3. Tags can be selected from the tag list
4. The correct saved-tag snackbar appears

### 30.6 About And Shutdown

1. About Device shows the specified firmware, serial, and open-hardware lines
2. Powering off shows the shutdown message and logo before screen off

## 31. Explicit Non-Goals For This Specification

This document does not define:

1. Mobile app route review UI
2. BLE app UX
3. Wi-Fi provisioning UX outside the on-device menu labels above
4. Detailed storage schema for tags
5. Calibration screens
6. OTA update screens

Those may be specified later, but they are not required to implement the on-device UI described here.

## Appendix A. Native 122 x 250 Pixel Tables

All coordinates in this appendix are final integer native-display coordinates derived from the simulator reference by rounding from the logical `144 x 296` grid.

### A.1 Home Dashboard, Default `none` State

| Element | x | y | w | h | Text anchor / baseline | Font |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| Status divider | 0 | 19 | 122 | 1 | - | - |
| Left status cluster anchor | 3 | 2 | - | - | group origin | icon bitmap / `6x10` |
| Tracking dot anchor | 98 | 10 | 5 | 5 | center at `(98, 10)` approx | bitmap |
| Battery anchor | 99 | 2 | 19 | 12 | group origin | existing firmware |
| PM block background | 0 | 27 | 122 | 44 | - | - |
| PM label | - | - | - | - | centered, baseline `y=41` | `6x10` |
| PM value | - | - | - | - | centered, baseline `y=68` | `10x20` |
| CO2 block background | 0 | 74 | 122 | 47 | - | - |
| CO2 label | - | - | - | - | centered, baseline `y=90` | `6x10` |
| CO2 value | - | - | - | - | centered, baseline `y=117` | `10x20` |
| Main divider | 0 | 127 | 122 | 2 | - | - |
| Vertical grid divider | 61 | 133 | 1 | 86 | - | - |
| Grid horizontal divider 1 | 0 | 162 | 122 | 1 | - | - |
| Grid horizontal divider 2, no chart | 0 | 191 | 122 | 1 | - | - |
| Bottom line above logo | 0 | 220 | 122 | 1 | - | - |
| Bottom logo | 0 | 221 | 122 | 24 | centered vertically in box | bitmap |
| Snackbar background | 0 | 232 | 122 | 18 | - | - |
| Snackbar text | - | - | - | - | centered, baseline about `y=241` | `6x10` |

### A.2 Secondary Metrics Grid

| Cell | x | y | w | h | Label anchor | Value anchor |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| Temp | 1 | 134 | 59 | 27 | `x=10`, baseline `y=142` | `x=10`, baseline `y=155` |
| Humidity | 62 | 134 | 59 | 27 | `x=68`, baseline `y=142` | `x=68`, baseline `y=155` |
| TVOC | 1 | 163 | 59 | 27 | `x=10`, baseline `y=171` | `x=10`, baseline `y=184` |
| NOx | 62 | 163 | 59 | 27 | `x=68`, baseline `y=171` | `x=68`, baseline `y=184` |
| Pressure | 1 | 192 | 59 | 27 | `x=10`, baseline `y=199` | `x=10`, baseline `y=213` |
| Altitude | 62 | 192 | 59 | 27 | `x=68`, baseline `y=199` | `x=68`, baseline `y=213` |

### A.3 Chart Variant

When a metric is selected and chart mode is active:

| Element | x | y | w | h | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| Strong divider above chart | 0 | 190 | 122 | 2 | replaces the lighter no-chart divider |
| Plot box | 0 | 221 | 122 | 29 | replaces bottom logo area |
| Plot area | 3 | 225 | 115 | 22 | after internal padding |
| Left axis | 3 | 225 | 1 | 22 | line only |
| Bottom axis | 3 | 247 | 115 | 1 | line only |
| Min card | 1 | 192 | 59 | 27 | replaces Pressure |
| Max card | 62 | 192 | 59 | 27 | replaces Altitude |

### A.4 Overlay Family: Menu, Settings, Settings Choice, Confirm, Tag List

| Element | x | y | w | h | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| Main-menu overlay background | 0 | 128 | 122 | 122 | first-level menu only |
| Row 0 rect | 5 | 132 | 112 | 20 | usually `Exit` |
| Row 1 rect | 5 | 154 | 112 | 20 | usually `Back` |
| Row 2 rect | 5 | 176 | 112 | 20 | first content row |
| Row 3 rect | 5 | 198 | 112 | 20 | second content row |
| Row 4 rect | 5 | 220 | 112 | 20 | third content row |

Text anchors for rows:

1. Row 0 baseline: `x=10 y=142`
2. Row 1 baseline: `x=10 y=164`
3. Row 2 baseline: `x=10 y=186`
4. Row 3 baseline: `x=10 y=208`
5. Row 4 baseline: `x=10 y=230`

Second-level full-screen overlay:

1. Background: `x=0 y=24 w=122 h=226`
2. Row 0 rect: `x=5 y=25 w=112 h=20`
3. Row 1 rect: `x=5 y=47 w=112 h=20`
4. Row 2 rect: `x=5 y=69 w=112 h=20`
5. Row 3 rect: `x=5 y=91 w=112 h=20`
6. Row 4 rect: `x=5 y=113 w=112 h=20`
7. Row 5 rect: `x=5 y=135 w=112 h=20`
8. Row 6 rect: `x=5 y=157 w=112 h=20`
9. Row 7 rect: `x=5 y=179 w=112 h=20`
10. Row 8 rect: `x=5 y=201 w=112 h=20`

Second-level text baselines:

1. Row 0: `x=10 y=35`
2. Row 1: `x=10 y=57`
3. Row 2: `x=10 y=79`
4. Row 3: `x=10 y=101`
5. Row 4: `x=10 y=123`
6. Row 5: `x=10 y=145`
7. Row 6: `x=10 y=167`
8. Row 7: `x=10 y=189`
9. Row 8: `x=10 y=211`

Second-level separator below `Back` when used:

1. `x=7 y=69 w=108 h=1`
2. This separator is currently used on `Settings`, `Tag list`, and `About Device`

### A.5 Main Menu

Main menu uses the overlay family geometry above with these rows:

1. Row 0: `Exit Menu`
2. Row 1: `Start Tracking` or `Stop Tracking`
3. Row 2: `Add Tag`
4. Row 3: `Settings`
5. Row 4: `About Device`

When not tracking:

1. `Add Tag` stays in Row 2
2. It is rendered disabled
3. It is skipped by selection movement

### A.6 Display Off

Display Off reuses the home status bar and overlay geometry but changes the main body:

| Element | x | y | w | h | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| Status divider | 0 | 19 | 122 | 1 | same as home |
| Display-off logo | 0 | 113 | 122 | 24 | centered in body |
| Overlay background | 0 | 128 | 122 | 122 | same as menu overlay when menu is open |

### A.7 About Device

About Device reuses the overlay family top rows and adds static metadata:

| Element | x | y | w | h | Text baseline |
| --- | ---: | ---: | ---: | ---: | --- |
| Exit row | 5 | 25 | 112 | 20 | `x=10 y=35` |
| Back row | 5 | 47 | 112 | 20 | `x=10 y=57` |
| Separator below Back | 7 | 69 | 108 | 1 | line only |
| Title | 10 | - | - | - | `y=91` |
| Firmware line | 10 | - | - | - | `y=106` |
| Serial line | 10 | - | - | - | `y=120` |
| Open hardware line | 10 | - | - | - | `y=133` |

### A.8 Clear Data Confirmation

Confirm screen uses the overlay family geometry with:

1. Row 0: `Exit`
2. Row 1: `Back`
3. Row 2: `Clear Data?`
4. Row 3: `No`
5. Row 4: `Yes`

### A.9 Shutdown Screen

| Element | x | y | w | h | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| Primary text center | 61 | 115 | - | - | `Powering off...` |
| Secondary text center | 61 | 135 | - | - | `See you soon` |
| Bottom logo | 0 | 218 | 122 | 24 | same width as display |

### A.10 Powered-Off Screen

Powered-off state:

1. Entire `122 x 250` display is blank
2. No status bar
3. No logo
4. No snackbar

## Appendix B. Proposed Value Formatting Contract

This appendix is a proposed formatting contract for review. It is intended to remove implementation guesswork.

### B.1 General Rules

1. Use ASCII punctuation and spacing in firmware strings unless the chosen `u8g2` font fully supports a Unicode glyph required by the label
2. If a value is invalid or unavailable, display `-`
3. For invalid values in secondary metrics, omit the unit and display only `-`
4. Use a single space between number and unit in lower-grid values
5. Hero values should not include the unit in the number line; units stay in the label line

### B.2 Metric Formatting Table

| Field | Display context | Format rule | Invalid rule | Overflow rule | Example |
| --- | --- | --- | --- | --- | --- |
| PM2.5 | Hero, `µg/m³` mode | `0.0` to `99.9` with 1 decimal; `100` and above as integer | `-` | `999+` above `999` | `0.7`, `12.4`, `118` |
| PM2.5 | Hero, `USAQI` mode | integer only | `-` | `500+` above `500` | `42`, `151` |
| CO2 | Hero | integer ppm | `-` | `9999+` above `9999` | `1682` |
| Temp | Lower grid | one decimal plus unit | `-` | integer fallback if width exceeded | `22.4 C`, `72.3 F` |
| Humidity | Lower grid | integer plus `%` | `-` | `99+ %` above two digits if needed | `48 %` |
| TVOC | Lower grid | one decimal, no extra unit | `-` | integer fallback if width exceeded | `3.4` |
| NOx | Lower grid | one decimal, no extra unit | `-` | integer fallback if width exceeded | `1.2` |
| Pressure | Lower grid | integer plus `hPa` | `-` | `9999+ hPa` if needed | `1013 hPa` |
| Altitude | Lower grid | integer plus `m` | `-` | `9999+ m` if needed | `520 m` |
| Chart Min/Max | Row 3 in chart mode | same formatting as selected metric | `-` | same as source metric | `0.4`, `1900`, `21.8 C` |

### B.3 Label Formatting

Hero labels:

1. `PM2.5 (ug/m3)` if firmware must stay ASCII-only
2. `PM2.5 (µg/m³)` if chosen font and rendering path support it
3. `PM2.5 (USAQI)` in AQI mode
4. `CO2 (ppm)`

Secondary labels:

1. `Temp`
2. `Humidity`
3. `TVOC`
4. `NOx`
5. `Pressure`
6. `Altitude`
7. `Min`
8. `Max`

### B.4 Text Overflow Policy

For all single-line UI rows and values:

1. No multi-line wrapping
2. No marquee scrolling
3. No horizontal animation
4. Use pre-shortened labels in the source strings
5. If a dynamic value still exceeds the field width, use the overflow fallback defined in the table above

## Appendix C. Proposed Runtime And Interaction Matrix

This appendix is a proposed runtime/interaction contract for review.

### C.1 Runtime State Matrix

| Runtime state | Visible screen | Buttons allowed | Menu available | Notes |
| --- | --- | --- | --- | --- |
| `IDLE` | full home UI | yes | yes | default interactive state |
| `TRACKING` | full home UI | yes | yes | tracking icon active; `Add Tag` enabled |
| `SYNC` | home dashboard with sync status | proposed: no navigation | proposed: no | keep screen readable, block conflicting interactions |
| `INACTIVE` | blank / low-power display | wake only | no | wake returns to `IDLE` |
| `SHUTDOWN` | shutdown screen, then blank | no | no | no interaction accepted |

### C.2 Proposed Transition Rules

1. Wake from `INACTIVE` returns to `IDLE`
2. Wake from `INACTIVE` returns to `main` UI mode
3. Wake from `INACTIVE` resets active metric to `none`
4. Wake from `INACTIVE` clears open menus
5. Full power-on resets to home, `none`, unlocked
6. Auto-lock returns to home before locking
7. Shutdown returns to home before showing shutdown messaging

### C.3 Interaction Guard Matrix

| Condition | Left short | Right short | Menu short | Menu long 2s |
| --- | --- | --- | --- | --- |
| Main, unlocked, normal display | previous metric | next metric | open main menu | toggle lock |
| Main, unlocked, display off | ignored | ignored | open main menu | toggle lock |
| Main, locked | snackbar | snackbar | snackbar | unlock |
| Menu/list screen, unlocked | previous row | next row | activate row | toggle lock |
| Menu/list screen, locked | snackbar | snackbar | snackbar | unlock |
| Shutdown screen | ignored | ignored | ignored | ignored |
| Powered off | ignored | ignored | ignored | ignored |

### C.4 Sync State Proposal

Because the current firmware may block other work during sync:

1. During active sync, show the dashboard with sync indicator visible
2. Ignore `Left`, `Right`, and `Menu` short presses
3. Ignore `Menu` long press during the critical blocking sync section
4. Return to `IDLE` home UI when sync completes

If product wants sync to remain interactive later, that should be specified as a separate change.

## Appendix D. List Screen Rendering Contract

This appendix defines one generic renderer contract for all list-based screens.

### D.1 Shared List Rules

1. Rows are single-line only
2. Row text is left aligned
3. Highlight is a filled rounded rectangle behind the row
4. Selected row text is light
5. Unselected row text is dark
6. Disabled row text is light gray
7. Disabled rows never receive the selection highlight

### D.2 Screen Contract Table

| Screen | Fixed rows | Content rows visible at once | Default selected row | Wrap, clamp, or page | Disabled rows |
| --- | --- | ---: | --- | --- | --- |
| Main menu | none | all rows visible | `Exit Menu` | wrap | `Add Tag` when not tracking |
| Settings | `Exit`, `Back` | 7 | `Back` | clamp with page jumps | none |
| Settings choice | `Exit`, `Back` | 7 | `Back` | wrap with scrolling window | none |
| Confirm | none beyond visible rows | all rows visible | `Back` | wrap | none |
| Tag list | `Exit`, `Back` | 7 | `Back` | clamp with page jumps | none |
| About | `Exit`, `Back` | no scrolling | `Back` | wrap | none |

### D.3 Page Rules For Settings And Tag List

For `Settings` and `Tag list`:

1. `Exit` and `Back` remain fixed at the top of the logical item model
2. The visible content area below them shows 7 rows at a time
3. Within a page, movement is one row at a time
4. Moving forward from the last visible content row on a page jumps to the next page and selects its first row
5. Moving backward from the first visible content row on a page jumps to the previous page and selects its last row
6. The page model clamps at the first and last page
7. Moving backward from the first content row on the first page returns selection to `Back`

### D.4 Scroll Rules For Settings Choice

For `Settings choice`:

1. `Exit` and `Back` remain fixed at the top of the logical item model
2. The visible content window below them shows 7 rows at a time
3. When the selected content row moves below the window, scroll the window down by one row
4. When the selected content row moves above the window, scroll the window up by one row
5. The window never scrolls past the final available content item
6. Selection wrapping remains enabled for the full option list

### D.5 Row Activation Rules

`Exit`

1. Returns immediately to the home screen

`Back`

1. Returns to the parent screen
2. Preserves the parent screen’s previous selection where practical

Disabled row

1. Cannot be selected
2. Cannot be activated
3. Is skipped by left/right navigation

Informational row such as `Clear Data?`

1. May be selectable in the current simulator model
2. Must not trigger an action when activated
