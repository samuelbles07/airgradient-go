<template>
  <svg
    viewBox="0 0 320 520"
    xmlns="http://www.w3.org/2000/svg"
    role="img"
    aria-label="AirGradient Go simulator"
  >
    <defs>
      <linearGradient id="ag-go-body-gradient" x1="0" y1="0" x2="0" y2="1">
        <stop offset="0%" stop-color="var(--main-white-color, #ffffff)" />
        <stop offset="100%" stop-color="var(--grayColor100, #f8f2eb)" />
      </linearGradient>

      <pattern id="ag-go-eink-pattern" width="6" height="6" patternUnits="userSpaceOnUse">
        <circle cx="1.5" cy="1.5" r="0.6" fill="rgba(0,0,0,0.06)" />
        <circle cx="4.5" cy="3.5" r="0.5" fill="rgba(0,0,0,0.05)" />
      </pattern>

      <filter id="ag-go-shadow" x="-20%" y="-20%" width="140%" height="140%">
        <feDropShadow dx="0" dy="10" stdDeviation="10" flood-color="#000" flood-opacity="0.18" />
      </filter>

      <filter id="ag-go-logo-black">
        <feColorMatrix
          type="matrix"
          values="
            0 0 0 0 0
            0 0 0 0 0
            0 0 0 0 0
            0 0 0 1 0
          "
        />
      </filter>
    </defs>

    <!-- Device photo render (provided by user in /public/images/go-render.jpg) -->
    <g filter="url(#ag-go-shadow)">
      <image
        href="/images/go-render.jpg"
        x="0"
        y="0"
        width="320"
        height="520"
        preserveAspectRatio="xMidYMid meet"
      />
    </g>

    <!-- Display UI canvas (122×250 px), aligned to the white display area in the render -->
    <g class="ag-go-display" :transform="displayTransform">
      <rect x="0" y="0" width="122" height="250" fill="var(--grayColor200, #eeede4)" />
      <rect x="0" y="0" width="122" height="250" fill="url(#ag-go-eink-pattern)" />

      <g :transform="displayContentTransform">
      <template v-if="powerPhase === 'on'">
        <!-- Toolbar -->
        <g aria-label="Status toolbar">
          <rect x="0" y="22" width="144" height="1" fill="rgba(0, 0, 0, 0.08)" />

          <!-- Tracking indicator (shown during active tracking) -->
          <g v-if="trackingActive" aria-label="Tracking">
            <circle
              cx="116"
              cy="12"
              r="2.8"
              fill="none"
              stroke="rgba(0, 0, 0, 0.68)"
              stroke-width="1.2"
            />
            <circle cx="116" cy="12" r="1.3" fill="rgba(0, 0, 0, 0.68)" stroke="none" />
          </g>

          <g
            transform="translate(4 2) scale(1.2)"
            fill="rgba(0, 0, 0, 0.68)"
            stroke="rgba(0, 0, 0, 0.68)"
            stroke-width="1.6"
            stroke-linecap="round"
            stroke-linejoin="round"
          >
            <!-- Lock -->
            <g transform="translate(0 0)" aria-label="Button lock" :opacity="deviceLocked ? 1 : 0.25">
              <rect x="1.5" y="7" width="7" height="5.8" rx="1.2" fill="none" />
              <path d="M3.1 7V5.7C3.1 4.2 4.2 3 5.7 3S8.3 4.2 8.3 5.7V7" fill="none" />
            </g>

            <!-- BLE -->
            <g transform="translate(12 0)" aria-label="BLE" :opacity="settings.bluetooth === 'on' ? 1 : 0.25">
              <text
                x="0"
                y="11"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="10"
                font-weight="700"
                fill="rgba(0, 0, 0, 0.68)"
                stroke="none"
                letter-spacing="0.3"
              >
                BLE
              </text>
            </g>

            <!-- WiFi -->
            <g transform="translate(32 0)" aria-label="WiFi" :opacity="settings.wifi === 'on' ? 1 : 0.25">
              <path d="M2 6.5C4.6 4.2 9.4 4.2 12 6.5" fill="none" />
              <path d="M3.7 8.4C5.3 7.0 8.7 7.0 10.3 8.4" fill="none" />
              <circle cx="7" cy="11" r="1.2" stroke="none" />
            </g>

            <!-- GPS -->
            <g transform="translate(45 0)" aria-label="GPS" :opacity="settings.gpsMode === 'off' ? 0.25 : 1">
              <path
                d="M2.4 2.6L13 7.2L6.2 12.2L7.6 7.4L2.4 2.6Z"
                fill="rgba(0, 0, 0, 0.68)"
                stroke="none"
              />
            </g>

          </g>

          <!-- Battery (outline + level fill) -->
          <g
            transform="translate(117 2) scale(1.2)"
            aria-label="Battery"
            fill="rgba(0, 0, 0, 0.68)"
            stroke="rgba(0, 0, 0, 0.68)"
            stroke-width="1.6"
            stroke-linecap="round"
            stroke-linejoin="round"
          >
            <rect x="0.8" y="3.2" width="18.8" height="9.6" rx="1.6" fill="none" />
            <rect x="20.2" y="5.8" width="1.8" height="4.4" rx="0.8" fill="rgba(0, 0, 0, 0.68)" stroke="none" />
            <rect x="2.2" y="4.6" width="12.8" height="6.8" rx="1.2" fill="rgba(0, 0, 0, 0.68)" stroke="none" />
          </g>
        </g>

        <template v-if="!isDisplayOff">
        <!-- PM2.5 -->
        <g aria-label="PM2.5 value">
          <rect
            x="0"
            y="32"
            width="144"
            height="52"
            rx="0"
            :fill="isPmInverted ? 'rgba(0, 0, 0, 0.78)' : 'none'"
          />
          <text
            x="72"
            y="48"
            text-anchor="middle"
            font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="14"
            font-weight="600"
            :fill="isPmInverted ? 'rgba(255, 255, 255, 0.78)' : 'rgba(0, 0, 0, 0.6)'"
          >
            {{ pmLabel }}
          </text>
          <text
            x="72"
            y="80"
            text-anchor="middle"
            font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="34"
            font-weight="700"
            :fill="isPmInverted ? 'rgba(255, 255, 255, 0.92)' : 'rgba(0, 0, 0, 0.78)'"
          >
            {{ pmValueDisplay }}
          </text>
        </g>

        <!-- CO2 (inverted) -->
        <g aria-label="CO2 value">
          <rect
            x="0"
            y="88"
            width="144"
            height="56"
            rx="0"
            :fill="
              isCo2Inverted
                ? 'rgba(0, 0, 0, 0.78)'
                : props.activeMetric === 'pm25'
                  ? 'rgba(0, 0, 0, 0.04)'
                  : 'none'
            "
          />
          <text
            x="72"
            y="106"
            text-anchor="middle"
            font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="14"
            font-weight="600"
            :fill="isCo2Inverted ? 'rgba(255, 255, 255, 0.78)' : 'rgba(0, 0, 0, 0.6)'"
          >
            CO2 (ppm)
          </text>
          <text
            x="72"
            y="139"
            text-anchor="middle"
            font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="34"
            font-weight="700"
            :fill="isCo2Inverted ? 'rgba(255, 255, 255, 0.92)' : 'rgba(0, 0, 0, 0.78)'"
          >
            1682
          </text>
        </g>

        <rect x="0" y="150" width="144" height="2" fill="rgba(0, 0, 0, 0.12)" />

        <g v-if="uiMode !== 'main'" aria-label="Menu Overlay">
          <rect
            x="0"
            :y="isSecondaryView ? 24 : 152"
            width="144"
            :height="isSecondaryView ? 272 : 144"
            fill="var(--grayColor200, #eeede4)"
          />

          <g v-if="uiMode === 'menu'" aria-label="Main menu">
            <g v-for="(item, idx) in menuItems" :key="item.key">
              <rect
                x="6"
                :y="156 + idx * 26"
                width="132"
                height="24"
                rx="3"
                :fill="
                  idx === menuIndex && !item.disabled ? 'rgba(0, 0, 0, 0.78)' : 'none'
                "
              />
              <text
                x="12"
                :y="168 + idx * 26"
                dominant-baseline="middle"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="12"
                font-weight="700"
                :fill="
                  item.disabled
                    ? 'rgba(0, 0, 0, 0.28)'
                    : idx === menuIndex
                      ? 'rgba(255, 255, 255, 0.9)'
                      : 'rgba(0, 0, 0, 0.72)'
                "
              >
                {{ item.label }}
              </text>
            </g>
          </g>

          <g v-else-if="uiMode === 'settings'" aria-label="Settings">
            <g v-for="(row, rowIdx) in settingsRows" :key="row.key">
              <rect
                x="6"
                :y="secondaryRowTop(rowIdx)"
                width="132"
                height="24"
                rx="3"
                :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
              />
              <text
                x="12"
                :y="secondaryRowTextY(rowIdx)"
                dominant-baseline="middle"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="12"
                font-weight="700"
                :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
              >
                {{ row.label }}
              </text>
            </g>
            <rect x="8" :y="secondarySectionDividerY(1)" width="128" height="1" fill="rgba(0, 0, 0, 0.18)" />
          </g>

          <g v-else-if="uiMode === 'settings-choice'" aria-label="Settings choice">
            <g v-for="(row, rowIdx) in settingsChoiceRows" :key="row.key">
              <rect
                x="6"
                :y="secondaryRowTop(rowIdx)"
                width="132"
                height="24"
                rx="3"
                :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
              />
              <text
                x="12"
                :y="secondaryRowTextY(rowIdx)"
                dominant-baseline="middle"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="12"
                font-weight="700"
                :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
              >
                {{ row.label }}
              </text>
            </g>
          </g>

          <g v-else-if="uiMode === 'about'" aria-label="About device">
            <g v-for="(row, rowIdx) in aboutRows" :key="row.key">
              <rect
                x="6"
                :y="secondaryRowTop(rowIdx)"
                width="132"
                height="24"
                rx="3"
                :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
              />
              <text
                x="12"
                :y="secondaryRowTextY(rowIdx)"
                dominant-baseline="middle"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="12"
                font-weight="700"
                :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
              >
                {{ row.label }}
              </text>
            </g>
            <rect x="8" :y="secondarySectionDividerY(1)" width="128" height="1" fill="rgba(0, 0, 0, 0.18)" />

            <text
              x="12"
              y="108"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="12"
              font-weight="700"
              fill="rgba(0, 0, 0, 0.78)"
            >
              AirGradient Go
            </text>
            <text
              x="12"
              y="126"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="10"
              font-weight="600"
              fill="rgba(0, 0, 0, 0.62)"
            >
              Firmware v0.4.0
            </text>
            <text
              x="12"
              y="142"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="10"
              font-weight="600"
              fill="rgba(0, 0, 0, 0.62)"
            >
              Serial 7A3C91F04B2D
            </text>
            <text
              x="12"
              y="158"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="10"
              font-weight="600"
              fill="rgba(0, 0, 0, 0.62)"
            >
              Open Source Hardware
            </text>
          </g>

          <g v-else-if="uiMode === 'confirm'" aria-label="Confirm dialog">
            <g v-for="(row, rowIdx) in confirmRows" :key="row.key">
              <rect
                x="6"
                :y="secondaryRowTop(rowIdx)"
                width="132"
                height="24"
                rx="3"
                :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
              />
              <text
                x="12"
                :y="secondaryRowTextY(rowIdx)"
                dominant-baseline="middle"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="12"
                font-weight="700"
                :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
              >
                {{ row.label }}
              </text>
            </g>
          </g>

          <g v-else-if="uiMode === 'tag-list'" aria-label="Source tags">
            <g v-for="(row, rowIdx) in tagListRows" :key="row.key">
              <rect
                x="6"
                :y="secondaryRowTop(rowIdx)"
                width="132"
                height="24"
                rx="3"
                :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
              />
              <text
                x="12"
                :y="secondaryRowTextY(rowIdx)"
                dominant-baseline="middle"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="12"
                font-weight="700"
                :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
              >
                {{ row.label }}
              </text>
            </g>
            <rect x="8" :y="secondarySectionDividerY(1)" width="128" height="1" fill="rgba(0, 0, 0, 0.18)" />
          </g>
        </g>

        <g v-else>
          <!-- Secondary metrics grid lines -->
          <g aria-hidden="true">
            <rect x="72" y="158" width="1" height="102" fill="rgba(0, 0, 0, 0.06)" />
            <rect x="0" y="192" width="144" height="1" fill="rgba(0, 0, 0, 0.06)" />
	            <rect
                x="0"
                :y="showChart ? 225 : 226"
                width="144"
                :height="showChart ? 2 : 1"
                :fill="showChart ? 'rgba(0, 0, 0, 0.12)' : 'rgba(0, 0, 0, 0.06)'"
              />
	            <rect x="0" y="260" width="144" height="1" fill="rgba(0, 0, 0, 0.06)" />
	          </g>

	          <!-- Secondary metrics -->
          <g aria-label="Secondary metrics">
          <!-- Row 1 -->
          <rect
            x="1"
            y="159"
            width="70"
            height="32"
            :fill="isBottomSelected('temp') ? 'rgba(0, 0, 0, 0.78)' : 'none'"
          />
          <text
            x="12"
            y="168"
            font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="10"
            font-weight="600"
            :fill="isBottomSelected('temp') ? 'rgba(255, 255, 255, 0.78)' : 'rgba(0, 0, 0, 0.55)'"
          >
            Temp
          </text>
          <text
            x="12"
            y="184"
            font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="14"
            font-weight="700"
            :fill="isBottomSelected('temp') ? 'rgba(255, 255, 255, 0.92)' : 'rgba(0, 0, 0, 0.78)'"
          >
            {{ tempValueDisplay }}
          </text>

          <rect
            x="73"
            y="159"
            width="70"
            height="32"
            :fill="isBottomSelected('humidity') ? 'rgba(0, 0, 0, 0.78)' : 'none'"
          />
          <text
            x="80"
            y="168"
            font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="10"
            font-weight="600"
            :fill="isBottomSelected('humidity') ? 'rgba(255, 255, 255, 0.78)' : 'rgba(0, 0, 0, 0.55)'"
          >
            Humidity
          </text>
          <text
            x="80"
            y="184"
            font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="14"
            font-weight="700"
            :fill="
              isBottomSelected('humidity') ? 'rgba(255, 255, 255, 0.92)' : 'rgba(0, 0, 0, 0.78)'
            "
          >
            48 %
          </text>

          <!-- Row 2 -->
          <rect
            x="1"
            y="193"
            width="70"
            height="32"
            :fill="isBottomSelected('tvoc') ? 'rgba(0, 0, 0, 0.78)' : 'none'"
          />
          <text
            x="12"
            y="202"
            font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="10"
            font-weight="600"
            :fill="isBottomSelected('tvoc') ? 'rgba(255, 255, 255, 0.78)' : 'rgba(0, 0, 0, 0.55)'"
          >
            TVOC
          </text>
          <text
            x="12"
            y="218"
            font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="14"
            font-weight="700"
            :fill="isBottomSelected('tvoc') ? 'rgba(255, 255, 255, 0.92)' : 'rgba(0, 0, 0, 0.78)'"
          >
            3.4
          </text>

          <rect
            x="73"
            y="193"
            width="70"
            height="32"
            :fill="isBottomSelected('nox') ? 'rgba(0, 0, 0, 0.78)' : 'none'"
          />
          <text
            x="80"
            y="202"
            font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="10"
            font-weight="600"
            :fill="isBottomSelected('nox') ? 'rgba(255, 255, 255, 0.78)' : 'rgba(0, 0, 0, 0.55)'"
          >
            NOx
          </text>
          <text
            x="80"
            y="218"
            font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
            font-size="14"
            font-weight="700"
            :fill="isBottomSelected('nox') ? 'rgba(255, 255, 255, 0.92)' : 'rgba(0, 0, 0, 0.78)'"
          >
            1.2
          </text>

          <!-- Row 3 -->
          <g v-if="!showChart" aria-label="Pressure">
            <rect
              x="1"
              y="227"
              width="70"
              height="32"
              :fill="isBottomSelected('pressure') ? 'rgba(0, 0, 0, 0.78)' : 'none'"
            />
            <text
              x="12"
              y="236"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="10"
              font-weight="600"
              :fill="
                isBottomSelected('pressure') ? 'rgba(255, 255, 255, 0.78)' : 'rgba(0, 0, 0, 0.55)'
              "
            >
              Pressure
            </text>
            <text
              x="12"
              y="252"
              font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="14"
              font-weight="700"
              :fill="
                isBottomSelected('pressure') ? 'rgba(255, 255, 255, 0.92)' : 'rgba(0, 0, 0, 0.78)'
              "
            >
              1013 hPa
            </text>
          </g>
          <g v-else aria-label="Chart minimum">
            <rect x="1" y="227" width="70" height="32" fill="none" />
            <text
              x="12"
              y="236"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="10"
              font-weight="600"
              fill="rgba(0, 0, 0, 0.55)"
            >
              Min
            </text>
            <text
              x="12"
              y="252"
              font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="14"
              font-weight="700"
              fill="rgba(0, 0, 0, 0.78)"
            >
              <tspan>{{ chartMinParts.num }}</tspan>
              <tspan
                v-if="chartMinParts.unit"
                font-size="10"
                font-weight="700"
                fill="rgba(0, 0, 0, 0.68)"
              >
                {{ chartMinParts.unit }}
              </tspan>
            </text>
          </g>

          <g v-if="!showChart" aria-label="Altitude">
            <rect
              x="73"
              y="227"
              width="70"
              height="32"
              :fill="isBottomSelected('altitude') ? 'rgba(0, 0, 0, 0.78)' : 'none'"
            />
            <text
              x="80"
              y="236"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="10"
              font-weight="600"
              :fill="
                isBottomSelected('altitude') ? 'rgba(255, 255, 255, 0.78)' : 'rgba(0, 0, 0, 0.55)'
              "
            >
              Altitude
            </text>
            <text
              x="80"
              y="252"
              font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="14"
              font-weight="700"
              :fill="
                isBottomSelected('altitude') ? 'rgba(255, 255, 255, 0.92)' : 'rgba(0, 0, 0, 0.78)'
              "
            >
              520 m
	            </text>
          </g>
          <g v-else aria-label="Chart maximum">
            <rect x="73" y="227" width="70" height="32" fill="none" />
            <text
              x="80"
              y="236"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="10"
              font-weight="600"
              fill="rgba(0, 0, 0, 0.55)"
            >
              Max
            </text>
            <text
              x="80"
              y="252"
              font-family="var(--primary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              font-size="14"
              font-weight="700"
              fill="rgba(0, 0, 0, 0.78)"
            >
              <tspan>{{ chartMaxParts.num }}</tspan>
              <tspan
                v-if="chartMaxParts.unit"
                font-size="10"
                font-weight="700"
                fill="rgba(0, 0, 0, 0.68)"
              >
                {{ chartMaxParts.unit }}
              </tspan>
            </text>
          </g>
          </g>

          <g v-if="showChart" aria-label="Selected metric chart">
	          <line
	            :x1="chartAxis.x"
	            :y1="chartAxis.yTop"
            :x2="chartAxis.x"
            :y2="chartAxis.yBottom"
            stroke="rgba(0, 0, 0, 0.12)"
            stroke-width="1"
          />
          <line
            :x1="chartAxis.x"
            :y1="chartAxis.yBottom"
            :x2="chartAxis.xRight"
            :y2="chartAxis.yBottom"
            stroke="rgba(0, 0, 0, 0.12)"
            stroke-width="1"
          />
          <polyline
            :points="chartPoints"
            fill="none"
            stroke="rgba(0, 0, 0, 0.65)"
            stroke-width="1.6"
	            stroke-linecap="round"
	            stroke-linejoin="round"
            />
          </g>
          <g v-else aria-label="AirGradient logo">
            <image
              x="0"
              y="262"
              width="144"
              height="28"
              href="/images/logos/logo.svg"
              preserveAspectRatio="xMidYMid meet"
              filter="url(#ag-go-logo-black)"
              opacity="0.75"
            />
          </g>

	          <g v-if="showFeedback" aria-label="Feedback message">
	            <rect x="0" y="275" width="144" height="21" fill="rgba(0, 0, 0, 0.78)" />
	            <text
              x="72"
              y="285.5"
              text-anchor="middle"
              dominant-baseline="middle"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              :font-size="feedbackFontSize"
              font-weight="700"
              fill="rgba(255, 255, 255, 0.92)"
            >
	              {{ feedbackMessage }}
	            </text>
	          </g>
	        </g>
	        </template>
        <template v-else>
          <image
            x="0"
            y="134"
            width="144"
            height="28"
            href="/images/logos/logo.svg"
            preserveAspectRatio="xMidYMid meet"
            filter="url(#ag-go-logo-black)"
            opacity="0.75"
          />

          <g v-if="uiMode !== 'main'" aria-label="Menu Overlay">
            <rect
              x="0"
              :y="isSecondaryView ? 24 : 152"
              width="144"
              :height="isSecondaryView ? 272 : 144"
              fill="var(--grayColor200, #eeede4)"
            />

            <g v-if="uiMode === 'menu'" aria-label="Main menu">
              <g v-for="(item, idx) in menuItems" :key="item.key">
                <rect
                  x="6"
                  :y="156 + idx * 26"
                  width="132"
                  height="24"
                  rx="3"
                  :fill="
                    idx === menuIndex && !item.disabled ? 'rgba(0, 0, 0, 0.78)' : 'none'
                  "
                />
                <text
                  x="12"
                  :y="168 + idx * 26"
                  dominant-baseline="middle"
                  font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                  font-size="12"
                  font-weight="700"
                  :fill="
                    item.disabled
                      ? 'rgba(0, 0, 0, 0.28)'
                      : idx === menuIndex
                        ? 'rgba(255, 255, 255, 0.9)'
                        : 'rgba(0, 0, 0, 0.72)'
                  "
                >
                  {{ item.label }}
                </text>
              </g>
            </g>

            <g v-else-if="uiMode === 'settings'" aria-label="Settings">
              <g v-for="(row, rowIdx) in settingsRows" :key="row.key">
                <rect
                  x="6"
                  :y="secondaryRowTop(rowIdx)"
                  width="132"
                  height="24"
                  rx="3"
                  :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
                />
                <text
                  x="12"
                  :y="secondaryRowTextY(rowIdx)"
                  dominant-baseline="middle"
                  font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                  font-size="12"
                  font-weight="700"
                  :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
                >
                  {{ row.label }}
                </text>
              </g>
              <rect x="8" :y="secondarySectionDividerY(1)" width="128" height="1" fill="rgba(0, 0, 0, 0.18)" />
            </g>

            <g v-else-if="uiMode === 'settings-choice'" aria-label="Settings choice">
              <g v-for="(row, rowIdx) in settingsChoiceRows" :key="row.key">
                <rect
                  x="6"
                  :y="secondaryRowTop(rowIdx)"
                  width="132"
                  height="24"
                  rx="3"
                  :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
                />
                <text
                  x="12"
                  :y="secondaryRowTextY(rowIdx)"
                  dominant-baseline="middle"
                  font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                  font-size="12"
                  font-weight="700"
                  :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
                >
                  {{ row.label }}
                </text>
              </g>
            </g>

            <g v-else-if="uiMode === 'about'" aria-label="About device">
              <g v-for="(row, rowIdx) in aboutRows" :key="row.key">
                <rect
                  x="6"
                  :y="secondaryRowTop(rowIdx)"
                  width="132"
                  height="24"
                  rx="3"
                  :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
                />
                <text
                  x="12"
                  :y="secondaryRowTextY(rowIdx)"
                  dominant-baseline="middle"
                  font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                  font-size="12"
                  font-weight="700"
                  :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
                >
                  {{ row.label }}
                </text>
              </g>
              <rect x="8" :y="secondarySectionDividerY(1)" width="128" height="1" fill="rgba(0, 0, 0, 0.18)" />

              <text
                x="12"
                y="108"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="12"
                font-weight="700"
                fill="rgba(0, 0, 0, 0.78)"
              >
                AirGradient Go
              </text>
              <text
                x="12"
                y="126"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="10"
                font-weight="600"
                fill="rgba(0, 0, 0, 0.62)"
              >
                Firmware v0.4.0
              </text>
              <text
                x="12"
                y="142"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="10"
                font-weight="600"
                fill="rgba(0, 0, 0, 0.62)"
              >
                Serial 7A3C91F04B2D
              </text>
              <text
                x="12"
                y="158"
                font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                font-size="10"
                font-weight="600"
                fill="rgba(0, 0, 0, 0.62)"
              >
                Open Source Hardware
              </text>
            </g>

            <g v-else-if="uiMode === 'confirm'" aria-label="Confirm dialog">
              <g v-for="(row, rowIdx) in confirmRows" :key="row.key">
                <rect
                  x="6"
                  :y="secondaryRowTop(rowIdx)"
                  width="132"
                  height="24"
                  rx="3"
                  :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
                />
                <text
                  x="12"
                  :y="secondaryRowTextY(rowIdx)"
                  dominant-baseline="middle"
                  font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                  font-size="12"
                  font-weight="700"
                  :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
                >
                  {{ row.label }}
                </text>
              </g>
            </g>

            <g v-else-if="uiMode === 'tag-list'" aria-label="Source tags">
              <g v-for="(row, rowIdx) in tagListRows" :key="row.key">
                <rect
                  x="6"
                  :y="secondaryRowTop(rowIdx)"
                  width="132"
                  height="24"
                  rx="3"
                  :fill="row.selected ? 'rgba(0, 0, 0, 0.78)' : 'none'"
                />
                <text
                  x="12"
                  :y="secondaryRowTextY(rowIdx)"
                  dominant-baseline="middle"
                  font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
                  font-size="12"
                  font-weight="700"
                  :fill="row.selected ? 'rgba(255, 255, 255, 0.9)' : 'rgba(0, 0, 0, 0.72)'"
                >
                  {{ row.label }}
                </text>
              </g>
              <rect x="8" :y="secondarySectionDividerY(1)" width="128" height="1" fill="rgba(0, 0, 0, 0.18)" />
            </g>
          </g>

          <g v-if="showFeedback" aria-label="Feedback message">
            <rect x="0" y="275" width="144" height="21" fill="rgba(0, 0, 0, 0.78)" />
            <text
              x="72"
              y="285.5"
              text-anchor="middle"
              dominant-baseline="middle"
              font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
              :font-size="feedbackFontSize"
              font-weight="700"
              fill="rgba(255, 255, 255, 0.92)"
            >
              {{ feedbackMessage }}
            </text>
          </g>
        </template>
      </template>

      <g v-else-if="powerPhase === 'shutting-down'" aria-label="Powering off">
        <text
          x="72"
          y="136"
          text-anchor="middle"
          font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
          font-size="14"
          font-weight="700"
          fill="rgba(0, 0, 0, 0.72)"
        >
          Powering off...
        </text>
        <text
          x="72"
          y="160"
          text-anchor="middle"
          font-family="var(--secondary-font, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif)"
          font-size="10"
          font-weight="600"
          fill="rgba(0, 0, 0, 0.54)"
        >
          See you soon
        </text>
        <image
          x="0"
          y="258"
          width="144"
          height="28"
          href="/images/logos/logo.svg"
          preserveAspectRatio="xMidYMid meet"
          filter="url(#ag-go-logo-black)"
          opacity="0.75"
        />
      </g>
      </g>
    </g>

    <!-- Touch buttons -->
	    <g class="ag-go-buttons">
        <!-- Prev -->
        <g
          class="ag-go-button"
          :class="{ 'ag-go-button--pressed': pressedButton === 'prev' }"
          style="cursor: pointer"
          role="button"
          aria-label="Previous metric"
          tabindex="0"
          @pointerdown="handleButtonPointerDown('prev')"
          @pointerup="handleButtonPointerUp('prev')"
          @pointercancel="handleButtonPointerCancel('prev')"
          @pointerleave="handleButtonPointerCancel('prev')"
          @click="handlePrev"
        >
          <rect
            x="42"
            y="118"
            width="44"
            height="44"
            rx="4"
            fill="rgba(160, 160, 160, 0.5)"
            stroke="rgba(60, 60, 60, 0.9)"
            stroke-width="2"
          />
          <polygon
            points="68,132 56,140 68,148"
            fill="rgba(60, 60, 60, 0.72)"
            stroke="none"
            pointer-events="none"
          />
        </g>

        <!-- Select -->
        <g
          class="ag-go-button"
          :class="{ 'ag-go-button--pressed': pressedButton === 'menu' }"
          style="cursor: pointer"
          role="button"
          aria-label="Menu / Select"
          tabindex="0"
          @pointerdown="handleButtonPointerDown('menu')"
          @pointerup="handleButtonPointerUp('menu')"
          @pointercancel="handleButtonPointerCancel('menu')"
          @pointerleave="handleButtonPointerCancel('menu')"
          @click="handleMenu"
        >
          <rect
            x="42"
            y="184"
            width="44"
            height="44"
            rx="4"
            fill="rgba(160, 160, 160, 0.5)"
            stroke="rgba(60, 60, 60, 0.9)"
            stroke-width="2"
          />
          <g
            stroke="rgba(60, 60, 60, 0.72)"
            stroke-width="2.2"
            stroke-linecap="round"
            pointer-events="none"
          >
            <line x1="54" y1="200" x2="74" y2="200" />
            <line x1="54" y1="206" x2="74" y2="206" />
            <line x1="54" y1="212" x2="74" y2="212" />
          </g>
        </g>

        <!-- Next -->
        <g
          class="ag-go-button"
          :class="{ 'ag-go-button--pressed': pressedButton === 'next' }"
          style="cursor: pointer"
          role="button"
          aria-label="Next metric"
          tabindex="0"
          @pointerdown="handleButtonPointerDown('next')"
          @pointerup="handleButtonPointerUp('next')"
          @pointercancel="handleButtonPointerCancel('next')"
          @pointerleave="handleButtonPointerCancel('next')"
          @click="handleNext"
        >
          <rect
            x="234"
            y="118"
            width="44"
            height="44"
            rx="4"
            fill="rgba(160, 160, 160, 0.5)"
            stroke="rgba(60, 60, 60, 0.9)"
            stroke-width="2"
          />
          <polygon
            points="252,132 264,140 252,148"
            fill="rgba(60, 60, 60, 0.72)"
            stroke="none"
            pointer-events="none"
          />
        </g>
	    </g>

  </svg>
</template>

<script setup lang="ts">
  type ActiveMetric =
    | 'none'
    | 'co2'
    | 'pm25'
    | 'temp'
    | 'humidity'
    | 'tvoc'
    | 'nox'
    | 'pressure'
    | 'altitude';

  const props = withDefaults(defineProps<{ activeMetric?: ActiveMetric; powered?: boolean }>(), {
    activeMetric: 'none',
    powered: true
  });

  const emit = defineEmits<{
    (e: 'update:activeMetric', value: ActiveMetric): void;
    (e: 'power-phase-change', value: 'on' | 'shutting-down' | 'off'): void;
  }>();

  const cycleOrder: ActiveMetric[] = [
    'none',
    'pm25',
    'co2',
    'temp',
    'humidity',
    'tvoc',
    'nox'
  ];

  const nextSelection = () => {
    const current = props.activeMetric;
    const index = cycleOrder.indexOf(current);
    emit('update:activeMetric', index === -1 ? 'none' : cycleOrder[(index + 1) % cycleOrder.length]);
  };

  const prevSelection = () => {
    const current = props.activeMetric;
    const index = cycleOrder.indexOf(current);
    if (index === -1) {
      emit('update:activeMetric', 'none');
      return;
    }

    emit(
      'update:activeMetric',
      cycleOrder[(index - 1 + cycleOrder.length) % cycleOrder.length]
    );
  };

  const isCo2Inverted = computed(() => props.activeMetric === 'co2');
  const isPmInverted = computed(() => props.activeMetric === 'pm25');
  const isBottomSelected = (id: Exclude<ActiveMetric, 'none' | 'co2' | 'pm25'>) =>
    props.activeMetric === id;

  const pressedButton = ref<null | 'prev' | 'next' | 'menu'>(null);
  let lockToggleTimeout: ReturnType<typeof setTimeout> | null = null;
  let autoLockTimeout: ReturnType<typeof setTimeout> | null = null;
  let shutdownTimeout: ReturnType<typeof setTimeout> | null = null;
  const suppressMenuClick = ref(false);

  type UIMode =
    | 'main'
    | 'menu'
    | 'settings'
    | 'settings-choice'
    | 'about'
    | 'confirm'
    | 'tag-list';
  type ButtonId = 'prev' | 'next' | 'menu';
  type AutoLockSeconds = 0 | 10 | 30 | 60;
  type ToggleState = 'on' | 'off';
  type IntervalStep = '1s' | '10s' | '30s' | '60s' | '5m' | '15m' | '1h';
  type SensorIntervalOption = 'Off' | IntervalStep;
  type DisplayIntervalOption = 'Display Off' | IntervalStep;
  type ConnectionMode = 'stationary' | 'portable' | 'offline';

  const uiMode = ref<UIMode>('main');
  const powerPhase = ref<'on' | 'shutting-down' | 'off'>(props.powered ? 'on' : 'off');

  const menuIndex = ref(0);
  const trackingActive = ref(false);
  const deviceLocked = ref(false);

  const menuItems = computed(() => [
    { key: 'exit', label: 'Exit Menu', disabled: false },
    { key: 'tracking', label: trackingActive.value ? 'Stop Tracking' : 'Start Tracking', disabled: false },
    { key: 'add-tag', label: 'Add Tag', disabled: !trackingActive.value },
    { key: 'settings', label: 'Settings', disabled: false },
    { key: 'about', label: 'About Device', disabled: false }
  ]);

  const sourceTags = [
    'Traffic Emissions',
    'Road Dust',
    'Construction Work',
    'Biomass Burning',
    'Garbage Burning',
    'Factory Emissions',
    'Smoking/Vaping',
    'Cooking',
    'Paint/Solvents',
    'Other Pollution'
  ];

  const tagListIndex = ref(1);
  const tagScrollStart = ref(0);
  const lastSelectedTag = ref<string | null>(null);
  const feedbackMessage = ref<string | null>(null);
  let feedbackTimeout: ReturnType<typeof setTimeout> | null = null;

  type Settings = {
    units: 'C' | 'F';
    pmDisplay: 'ugm3' | 'usAqi';
    displayInterval: DisplayIntervalOption;
    pmInterval: SensorIntervalOption;
    otherSensorInterval: SensorIntervalOption;
    gpsMode: 'off' | 'tracking' | 'always';
    connectionMode: ConnectionMode;
    wifi: ToggleState;
    bluetooth: ToggleState;
    airplaneMode: ToggleState;
    autoLockSeconds: AutoLockSeconds;
  };

  const settings = ref<Settings>({
    units: 'C',
    pmDisplay: 'ugm3',
    displayInterval: '10s',
    pmInterval: '10s',
    otherSensorInterval: '10s',
    gpsMode: 'tracking',
    connectionMode: 'portable',
    wifi: 'off',
    bluetooth: 'on',
    airplaneMode: 'off',
    autoLockSeconds: 10
  });

  const tempValueC = 22.4;
  const tempValueDisplay = computed(() => {
    if (settings.value.units === 'F') {
      const f = tempValueC * (9 / 5) + 32;
      return `${f.toFixed(1)} °F`;
    }
    return `${tempValueC.toFixed(1)} °C`;
  });

  const pmValueUg = 0.7;
  const pmToUsAqi = (pm: number) => {
    const bps = [
      { cLow: 0.0, cHigh: 12.0, iLow: 0, iHigh: 50 },
      { cLow: 12.1, cHigh: 35.4, iLow: 51, iHigh: 100 },
      { cLow: 35.5, cHigh: 55.4, iLow: 101, iHigh: 150 },
      { cLow: 55.5, cHigh: 150.4, iLow: 151, iHigh: 200 },
      { cLow: 150.5, cHigh: 250.4, iLow: 201, iHigh: 300 },
      { cLow: 250.5, cHigh: 350.4, iLow: 301, iHigh: 400 },
      { cLow: 350.5, cHigh: 500.4, iLow: 401, iHigh: 500 }
    ];

    const clamped = Math.max(0, Math.min(pm, 500.4));
    const bp = bps.find((b) => clamped >= b.cLow && clamped <= b.cHigh) ?? bps[0];
    const aqi =
      ((bp.iHigh - bp.iLow) / (bp.cHigh - bp.cLow)) * (clamped - bp.cLow) + bp.iLow;
    return Math.round(aqi);
  };

  const pmLabel = computed(() =>
    settings.value.pmDisplay === 'ugm3' ? 'PM2.5 (µg/m³)' : 'PM2.5 (USAQI)'
  );
  const pmValueDisplay = computed(() => {
    if (settings.value.pmDisplay === 'usAqi') return String(pmToUsAqi(pmValueUg));
    return pmValueUg.toFixed(1);
  });

  const isDisplayOff = computed(() => settings.value.displayInterval === 'Display Off');
  const isSecondaryView = computed(() =>
    ['settings', 'settings-choice', 'about', 'confirm', 'tag-list'].includes(uiMode.value)
  );
  const showFeedback = computed(() => uiMode.value === 'main' && feedbackMessage.value !== null);
  const feedbackFontSize = computed(() => 10);

  const secondaryRowTop = (rowIdx: number) => 30 + rowIdx * 26;
  const secondaryRowTextY = (rowIdx: number) => 42 + rowIdx * 26;
  const secondarySectionDividerY = (rowIdx: number) => secondaryRowTop(rowIdx) + 26;

  const formatGpsMode = (mode: Settings['gpsMode']) => {
    switch (mode) {
      case 'off':
        return 'OFF';
      case 'tracking':
        return 'TRACKING';
      case 'always':
        return 'ALWAYS';
    }
  };

  const formatConnectionMode = (mode: ConnectionMode) => {
    switch (mode) {
      case 'stationary':
        return 'Stationary';
      case 'portable':
        return 'Portable';
      case 'offline':
        return 'Offline';
    }
  };

  const formatAutoLock = (seconds: AutoLockSeconds) => (seconds === 0 ? 'Off' : `${seconds}s`);

  const settingsRowIndex = ref(1);
  const settingsScrollStart = ref(0);
  const aboutIndex = ref(1);
  const settingsChoiceIndex = ref(1);
  const settingsChoiceScrollStart = ref(0);
  const editingSettingId = ref<
    | null
    | 'units'
    | 'pmDisplay'
    | 'displayInterval'
    | 'pmInterval'
    | 'otherSensorInterval'
    | 'gpsMode'
    | 'connectionMode'
    | 'autoLockSeconds'
  >(null);
  const confirmIndex = ref(1);

  const settingsItems = computed(() => [
    { key: 'units', label: `Units: °${settings.value.units}` },
    { key: 'pmDisplay', label: `PM Display: ${settings.value.pmDisplay === 'ugm3' ? 'µg/m³' : 'USAQI'}` },
    { key: 'displayInterval', label: `Display Interval: ${settings.value.displayInterval}` },
    { key: 'pmInterval', label: `PM Interval: ${settings.value.pmInterval}` },
    { key: 'otherSensorInterval', label: `Other Sensor Int.: ${settings.value.otherSensorInterval}` },
    { key: 'gpsMode', label: `GPS Mode: ${formatGpsMode(settings.value.gpsMode)}` },
    { key: 'connectionMode', label: `Mode: ${formatConnectionMode(settings.value.connectionMode)}` },
    { key: 'autoLockSeconds', label: `Auto Lock: ${formatAutoLock(settings.value.autoLockSeconds)}` },
    { key: 'clearData', label: 'Data: Clear Data' }
  ]);

  const syncWindow = (
    selectedIndex: number,
    optionCount: number,
    visible: number,
    scrollStart: { value: number }
  ) => {
    const optionIndex = selectedIndex - 1;
    if (selectedIndex <= 0 || selectedIndex >= optionCount + 1) return;
    const maxScrollStart = Math.max(0, optionCount - visible);
    if (optionIndex < scrollStart.value) scrollStart.value = optionIndex;
    if (optionIndex > scrollStart.value + (visible - 1)) scrollStart.value = optionIndex - (visible - 1);
    scrollStart.value = Math.min(scrollStart.value, maxScrollStart);
  };

  const settingsRows = computed(() => {
    const visible = 7;
    const items = settingsItems.value;
    const maxIndex = items.length + 1; // 0..(items+1) where 0=Exit, 1=Back, 2..=items
    const pageSize = visible;
    const pageStart = Math.floor(settingsScrollStart.value / pageSize) * pageSize;
    settingsRowIndex.value = Math.max(0, Math.min(maxIndex, settingsRowIndex.value));

    const rows: Array<{ key: string; label: string; selected: boolean }> = [
      { key: 'exit', label: 'Exit', selected: settingsRowIndex.value === 0 },
      { key: 'back', label: 'Back', selected: settingsRowIndex.value === 1 }
    ];

    for (let i = 0; i < visible; i += 1) {
      const idx = pageStart + i;
      if (idx >= items.length) break;
      rows.push({
        key: String(items[idx].key),
        label: items[idx].label,
        selected: settingsRowIndex.value === idx + 2
      });
    }

    return rows;
  });

  const getChoiceOptions = (id: NonNullable<typeof editingSettingId.value>) => {
    switch (id) {
      case 'units':
        return ['°C', '°F'];
      case 'pmDisplay':
        return ['µg/m³', 'USAQI'];
      case 'displayInterval':
        return ['1s', '10s', '30s', '60s', '5m', '15m', '1h', 'Display Off'];
      case 'pmInterval':
      case 'otherSensorInterval':
        return ['1s', '10s', '30s', '60s', '5m', '15m', '1h', 'Off'];
      case 'gpsMode':
        return ['Always Off', 'On When Tracking', 'Always On'];
      case 'connectionMode':
        return ['Stationary', 'Portable', 'Offline / Airplane Mode'];
      case 'autoLockSeconds':
        return ['Off', '10 Seconds', '30 Seconds', '60 Seconds'];
    }
  };

  const aboutRows = computed(() => [
    { key: 'exit', label: 'Exit', selected: aboutIndex.value === 0 },
    { key: 'back', label: 'Back', selected: aboutIndex.value === 1 }
  ]);

  const settingsChoiceRows = computed(() => {
    const visible = 7;
    const id = editingSettingId.value;
    const options = id ? getChoiceOptions(id) : [];
    const maxIndex = options.length + 1; // 0..(options+1) where 0=Exit, 1=Back, 2..=options
    settingsChoiceIndex.value = Math.max(0, Math.min(maxIndex, settingsChoiceIndex.value));
    syncWindow(settingsChoiceIndex.value - 1, options.length, visible, settingsChoiceScrollStart);

    const rows: Array<{ key: string; label: string; selected: boolean }> = [
      { key: 'exit', label: 'Exit', selected: settingsChoiceIndex.value === 0 },
      { key: 'back', label: 'Back', selected: settingsChoiceIndex.value === 1 }
    ];

    for (let i = 0; i < visible; i += 1) {
      const idx = settingsChoiceScrollStart.value + i;
      if (idx >= options.length) break;
      rows.push({
        key: `opt-${idx}`,
        label: options[idx],
        selected: settingsChoiceIndex.value === idx + 2
      });
    }

    return rows;
  });

  const confirmRows = computed(() => [
    { key: 'exit', label: 'Exit', selected: confirmIndex.value === 0 },
    { key: 'back', label: 'Back', selected: confirmIndex.value === 1 },
    { key: 'question', label: 'Clear Data?', selected: confirmIndex.value === 2 },
    { key: 'no', label: 'No', selected: confirmIndex.value === 3 },
    { key: 'yes', label: 'Yes', selected: confirmIndex.value === 4 }
  ]);

  const tagListRows = computed(() => {
    const visible = 7;
    const tags = sourceTags;
    const pageSize = visible;
    const pageStart = Math.floor(tagScrollStart.value / pageSize) * pageSize;
    const rows: Array<{ key: string; label: string; selected: boolean }> = [
      { key: 'exit', label: 'Exit', selected: tagListIndex.value === 0 },
      { key: 'back', label: 'Back', selected: tagListIndex.value === 1 }
    ];

    for (let i = 0; i < visible; i += 1) {
      const idx = pageStart + i;
      if (idx >= tags.length) break;
      rows.push({
        key: `source-${idx}`,
        label: tags[idx],
        selected: tagListIndex.value === idx + 2
      });
    }

    return rows;
  });

  const showChart = computed(() => uiMode.value === 'main' && props.activeMetric !== 'none');

  const chartRect = {
    x: 0,
    y: 262,
    w: 144,
    h: 34,
    padX: 4,
    padY: 4
  };

  const hashStringToSeed = (value: string) => {
    let hash = 2166136261;
    for (let i = 0; i < value.length; i += 1) {
      hash ^= value.charCodeAt(i);
      hash = Math.imul(hash, 16777619);
    }
    return hash >>> 0;
  };

  const mulberry32 = (seed: number) => {
    let t = seed >>> 0;
    return () => {
      t += 0x6d2b79f5;
      let x = Math.imul(t ^ (t >>> 15), 1 | t);
      x ^= x + Math.imul(x ^ (x >>> 7), 61 | x);
      return ((x ^ (x >>> 14)) >>> 0) / 4294967296;
    };
  };

  const chartSeries = computed(() => {
    if (!showChart.value) return '';

    const count = 30;
    const seed = hashStringToSeed(String(props.activeMetric || 'none'));
    const rand = mulberry32(seed);
    const phase = rand() * Math.PI * 2;

    const getMetricRange = () => {
      switch (props.activeMetric) {
        case 'co2':
          return { min: 400, max: 2000, decimals: 0 };
        case 'pm25':
          return settings.value.pmDisplay === 'usAqi'
            ? { min: 0, max: 200, decimals: 0 }
            : { min: 0, max: 50, decimals: 1 };
        case 'temp':
          return settings.value.units === 'F'
            ? { min: 59, max: 86, decimals: 1 }
            : { min: 15, max: 30, decimals: 1 };
        case 'humidity':
          return { min: 20, max: 80, decimals: 0 };
        case 'tvoc':
          return { min: 0, max: 10, decimals: 1 };
        case 'nox':
          return { min: 0, max: 10, decimals: 1 };
        case 'pressure':
          return { min: 980, max: 1030, decimals: 0 };
        case 'altitude':
          return { min: 0, max: 1500, decimals: 0 };
        default:
          return { min: 0, max: 1, decimals: 2 };
      }
    };

    const metric = getMetricRange();
    const span = metric.max - metric.min;

    const values: number[] = [];
    for (let i = 0; i < count; i += 1) {
      const t = i / (count - 1);
      const smooth = 0.5 + 0.32 * Math.sin(t * Math.PI * 2 * 1.2 + phase);
      const noise = (rand() - 0.5) * 0.12;
      const norm = Math.min(1, Math.max(0, smooth + noise));
      values.push(metric.min + norm * span);
    }

    const min = Math.min(...values);
    const max = Math.max(...values);
    const range = Math.max(1e-6, max - min);

    const innerW = chartRect.w - chartRect.padX * 2;
    const innerH = chartRect.h - chartRect.padY * 2;

    const points: string[] = [];
    for (let i = 0; i < count; i += 1) {
      const x = chartRect.x + chartRect.padX + (i / (count - 1)) * innerW;
      const yNorm = (values[i] - min) / range;
      const y = chartRect.y + chartRect.padY + (1 - yNorm) * innerH;
      points.push(`${x.toFixed(1)},${y.toFixed(1)}`);
    }

    return { points: points.join(' '), min, max, decimals: metric.decimals };
  });

  const chartAxis = computed(() => {
    const x = chartRect.x + chartRect.padX;
    const yTop = chartRect.y + chartRect.padY;
    const yBottom = chartRect.y + chartRect.h - chartRect.padY;
    const xRight = chartRect.x + chartRect.w - chartRect.padX;
    return { x, yTop, yBottom, xRight };
  });

  const chartPoints = computed(() => {
    const series = chartSeries.value;
    if (!series) return '';
    return series.points;
  });

  const chartMinMax = computed(() => {
    const series = chartSeries.value;
    if (!series) return null;
    return series;
  });

  const chartValueSuffix = computed(() => {
    switch (props.activeMetric) {
      case 'co2':
        return ' ppm';
      case 'pm25':
        return settings.value.pmDisplay === 'usAqi' ? ' AQI' : ' µg/m³';
      case 'temp':
        return settings.value.units === 'F' ? ' °F' : ' °C';
      case 'humidity':
        return ' %';
      case 'pressure':
        return ' hPa';
      case 'altitude':
        return ' m';
      default:
        return '';
    }
  });

  const formatChartValueParts = (value: number) => {
    const series = chartMinMax.value;
    const decimals = series?.decimals ?? 0;
    return { num: value.toFixed(decimals), unit: chartValueSuffix.value };
  };

  const chartMinParts = computed(() => {
    const series = chartMinMax.value;
    if (!series) return { num: '—', unit: '' };
    return formatChartValueParts(series.min);
  });

  const chartMaxParts = computed(() => {
    const series = chartMinMax.value;
    if (!series) return { num: '—', unit: '' };
    return formatChartValueParts(series.max);
  });

  // Real display resolution is 122×250 px. The white-area bbox detected in the render
  // stays the same, so we fit that smaller canvas to the same panel and scale the
  // existing 144×296 UI drawing into the new display resolution.
  const displayTransform = 'translate(106.476 105.247) scale(0.88682 0.9312)';
  const displayContentTransform = 'scale(0.847222 0.844595)';

  const wrapIndex = (next: number, count: number) => ((next % count) + count) % count;

  const getNextEnabledMenuIndex = (currentIndex: number, delta: 1 | -1) => {
    const items = menuItems.value;
    if (items.length === 0) return 0;

    let next = currentIndex;
    for (let i = 0; i < items.length; i += 1) {
      next = wrapIndex(next + delta, items.length);
      if (!items[next]?.disabled) return next;
    }

    return currentIndex;
  };

  const clearAutoLockTimeout = () => {
    if (autoLockTimeout) {
      clearTimeout(autoLockTimeout);
      autoLockTimeout = null;
    }
  };

  const clearShutdownTimeout = () => {
    if (shutdownTimeout) {
      clearTimeout(shutdownTimeout);
      shutdownTimeout = null;
    }
  };

  const flashLockedMessage = () => {
    flashFeedback('Long press Menu 2s to unlock');
  };

  const lockDevice = (message: string) => {
    deviceLocked.value = true;
    pressedButton.value = null;
    flashFeedback(message);
    clearAutoLockTimeout();
  };

  const unlockDevice = () => {
    deviceLocked.value = false;
    flashFeedback('Buttons unlocked');
  };

  const scheduleAutoLock = () => {
    clearAutoLockTimeout();
    if (deviceLocked.value || settings.value.autoLockSeconds === 0) return;

    autoLockTimeout = setTimeout(() => {
      if (!deviceLocked.value) {
        goToHomeScreen();
        lockDevice('Device auto-locked');
      }
    }, settings.value.autoLockSeconds * 1000);
  };

  const registerUserInteraction = () => {
    if (!deviceLocked.value && powerPhase.value === 'on') scheduleAutoLock();
  };

  const clearLockToggleTimeout = () => {
    if (lockToggleTimeout) {
      clearTimeout(lockToggleTimeout);
      lockToggleTimeout = null;
    }
  };

  const toggleDeviceLock = () => {
    if (deviceLocked.value) {
      unlockDevice();
      scheduleAutoLock();
      return;
    }

    lockDevice('Buttons locked');
  };

  const handleButtonPointerDown = (button: ButtonId) => {
    if (powerPhase.value !== 'on') return;
    pressedButton.value = button;

    if (button !== 'menu') {
      registerUserInteraction();
      return;
    }

    if (!deviceLocked.value) registerUserInteraction();

    clearLockToggleTimeout();
    lockToggleTimeout = setTimeout(() => {
      suppressMenuClick.value = true;
      toggleDeviceLock();
    }, 2000);
  };

  const handleButtonPointerUp = (button: ButtonId) => {
    if (powerPhase.value !== 'on') return;
    if (pressedButton.value === button) pressedButton.value = null;
    clearLockToggleTimeout();
  };

  const handleButtonPointerCancel = (button: ButtonId) => {
    if (powerPhase.value !== 'on') return;
    if (pressedButton.value === button) pressedButton.value = null;
    clearLockToggleTimeout();
  };

  const openMainMenu = () => {
    uiMode.value = 'menu';
    // Default highlight on "Exit Menu" (first row).
    menuIndex.value = 0;
  };

  const closeMenus = () => {
    uiMode.value = 'main';
  };

  const goToHomeScreen = () => {
    closeMenus();
    emit('update:activeMetric', 'none');
  };

  const openTagList = () => {
    uiMode.value = 'tag-list';
    tagScrollStart.value = 0;
    // Default highlight on "Back" (Exit is first).
    tagListIndex.value = 1;
  };

  const openAboutScreen = () => {
    uiMode.value = 'about';
    aboutIndex.value = 1;
  };

  const flashFeedback = (message: string) => {
    feedbackMessage.value = message;

    if (feedbackTimeout) clearTimeout(feedbackTimeout);
    feedbackTimeout = setTimeout(() => {
      feedbackMessage.value = null;
      feedbackTimeout = null;
    }, 3000);
  };

  const moveTagSelection = (delta: number) => {
    const tags = sourceTags;
    const maxIndex = tags.length + 1; // 0..(tags+1) where 0=Exit, 1=Back, 2..=tags
    const visible = 7;

    if (tagListIndex.value <= 1) {
      const next = Math.max(0, Math.min(maxIndex, tagListIndex.value + delta));
      tagListIndex.value = next;
      return;
    }

    const currentTagIdx = tagListIndex.value - 2;
    const currentPageStart = Math.floor(currentTagIdx / visible) * visible;
    const currentPageEnd = Math.min(currentPageStart + visible - 1, tags.length - 1);

    if (delta > 0) {
      if (currentTagIdx < currentPageEnd) {
        tagListIndex.value = Math.min(maxIndex, tagListIndex.value + 1);
        return;
      }

      const nextPageStart = currentPageStart + visible;
      if (nextPageStart < tags.length) {
        tagScrollStart.value = nextPageStart;
        tagListIndex.value = nextPageStart + 2;
      }
      return;
    }

    if (currentTagIdx > currentPageStart) {
      tagListIndex.value = Math.max(2, tagListIndex.value - 1);
      return;
    }

    const prevPageStart = currentPageStart - visible;
    if (prevPageStart >= 0) {
      const prevPageEnd = Math.min(prevPageStart + visible - 1, tags.length - 1);
      tagScrollStart.value = prevPageStart;
      tagListIndex.value = prevPageEnd + 2;
      return;
    }

    tagListIndex.value = 1;
  };

  const moveSettingsSelection = (delta: number) => {
    const items = settingsItems.value;
    const maxIndex = items.length + 1; // 0..(items+1) where 0=Exit, 1=Back, 2..=items
    const visible = 7;

    if (settingsRowIndex.value <= 1) {
      const next = Math.max(0, Math.min(maxIndex, settingsRowIndex.value + delta));
      settingsRowIndex.value = next;
      return;
    }

    const currentItemIdx = settingsRowIndex.value - 2;
    const currentPageStart = Math.floor(currentItemIdx / visible) * visible;
    const currentPageEnd = Math.min(currentPageStart + visible - 1, items.length - 1);

    if (delta > 0) {
      if (currentItemIdx < currentPageEnd) {
        settingsRowIndex.value = Math.min(maxIndex, settingsRowIndex.value + 1);
        return;
      }

      const nextPageStart = currentPageStart + visible;
      if (nextPageStart < items.length) {
        settingsScrollStart.value = nextPageStart;
        settingsRowIndex.value = nextPageStart + 2;
      }
      return;
    }

    if (currentItemIdx > currentPageStart) {
      settingsRowIndex.value = Math.max(2, settingsRowIndex.value - 1);
      return;
    }

    const prevPageStart = currentPageStart - visible;
    if (prevPageStart >= 0) {
      const prevPageEnd = Math.min(prevPageStart + visible - 1, items.length - 1);
      settingsScrollStart.value = prevPageStart;
      settingsRowIndex.value = prevPageEnd + 2;
      return;
    }

    settingsRowIndex.value = 1;
  };

  const applyConnectionMode = (mode: ConnectionMode) => {
    settings.value.connectionMode = mode;

    if (mode === 'stationary') {
      settings.value.bluetooth = 'off';
      settings.value.wifi = 'on';
      settings.value.airplaneMode = 'off';
      return;
    }

    if (mode === 'portable') {
      settings.value.bluetooth = 'on';
      settings.value.wifi = 'off';
      settings.value.airplaneMode = 'off';
      return;
    }

    settings.value.bluetooth = 'off';
    settings.value.wifi = 'off';
    settings.value.airplaneMode = 'on';
  };

  const handlePrev = () => {
    if (powerPhase.value !== 'on') return;

    if (deviceLocked.value) {
      flashLockedMessage();
      return;
    }

    registerUserInteraction();

    if (uiMode.value === 'menu') {
      menuIndex.value = getNextEnabledMenuIndex(menuIndex.value, -1);
      return;
    }

    if (uiMode.value === 'settings') {
      moveSettingsSelection(-1);
      return;
    }

    if (uiMode.value === 'about') {
      aboutIndex.value = wrapIndex(aboutIndex.value - 1, aboutRows.value.length);
      return;
    }

    if (uiMode.value === 'settings-choice') {
      const id = editingSettingId.value;
      const options = id ? getChoiceOptions(id) : [];
      settingsChoiceIndex.value = wrapIndex(settingsChoiceIndex.value - 1, options.length + 2);
      return;
    }

    if (uiMode.value === 'confirm') {
      confirmIndex.value = wrapIndex(confirmIndex.value - 1, confirmRows.value.length);
      return;
    }

    if (uiMode.value === 'tag-list') {
      moveTagSelection(-1);
      return;
    }

    if (isDisplayOff.value && uiMode.value === 'main') return;

    prevSelection();
  };

  const handleNext = () => {
    if (powerPhase.value !== 'on') return;

    if (deviceLocked.value) {
      flashLockedMessage();
      return;
    }

    registerUserInteraction();

    if (uiMode.value === 'menu') {
      menuIndex.value = getNextEnabledMenuIndex(menuIndex.value, 1);
      return;
    }

    if (uiMode.value === 'settings') {
      moveSettingsSelection(1);
      return;
    }

    if (uiMode.value === 'about') {
      aboutIndex.value = wrapIndex(aboutIndex.value + 1, aboutRows.value.length);
      return;
    }

    if (uiMode.value === 'settings-choice') {
      const id = editingSettingId.value;
      const options = id ? getChoiceOptions(id) : [];
      settingsChoiceIndex.value = wrapIndex(settingsChoiceIndex.value + 1, options.length + 2);
      return;
    }

    if (uiMode.value === 'confirm') {
      confirmIndex.value = wrapIndex(confirmIndex.value + 1, confirmRows.value.length);
      return;
    }

    if (uiMode.value === 'tag-list') {
      moveTagSelection(1);
      return;
    }

    if (isDisplayOff.value && uiMode.value === 'main') return;

    nextSelection();
  };

  const handleMenu = () => {
    if (powerPhase.value !== 'on') return;

    if (suppressMenuClick.value) {
      suppressMenuClick.value = false;
      return;
    }

    if (deviceLocked.value) {
      flashLockedMessage();
      return;
    }

    registerUserInteraction();

    if (uiMode.value === 'main') {
      openMainMenu();
      return;
    }

    if (uiMode.value === 'menu') {
      const selectedItem = menuItems.value[menuIndex.value];
      if (selectedItem?.disabled) {
        return;
      }

      const selected = selectedItem?.key;
      if (selected === 'add-tag') {
        openTagList();
        return;
      }

      if (selected === 'settings') {
        uiMode.value = 'settings';
        settingsRowIndex.value = 1;
        settingsScrollStart.value = 0;
        return;
      }

      if (selected === 'about') {
        openAboutScreen();
        return;
      }

      if (selected === 'tracking') {
        trackingActive.value = !trackingActive.value;
        flashFeedback(trackingActive.value ? 'Tracking started' : 'Tracking stopped');
      }

      closeMenus();
      return;
    }

    if (uiMode.value === 'settings') {
      const items = settingsItems.value;
      const maxIndex = items.length + 1; // 0=Exit, 1=Back, 2..=items

      if (settingsRowIndex.value === 0) {
        closeMenus();
        return;
      }

      if (settingsRowIndex.value === 1) {
        uiMode.value = 'menu';
        menuIndex.value = 0;
        return;
      }

      settingsRowIndex.value = Math.max(0, Math.min(maxIndex, settingsRowIndex.value));

      const key = items[settingsRowIndex.value - 2]?.key;
      if (key === 'clearData') {
        uiMode.value = 'confirm';
        confirmIndex.value = 1;
        return;
      }

      editingSettingId.value = key as NonNullable<typeof editingSettingId.value>;
      uiMode.value = 'settings-choice';
      settingsChoiceScrollStart.value = 0;
      // Default highlight on "Back" (Exit is first).
      settingsChoiceIndex.value = 1;

      return;
    }

    if (uiMode.value === 'settings-choice') {
      const id = editingSettingId.value;
      const options = id ? getChoiceOptions(id) : [];
      const maxIndex = options.length + 1; // 0=Exit, 1=Back, 2..=options

      if (settingsChoiceIndex.value === 0) {
        closeMenus();
        return;
      }

      if (settingsChoiceIndex.value === 1) {
        uiMode.value = 'settings';
        return;
      }

      settingsChoiceIndex.value = Math.max(0, Math.min(maxIndex, settingsChoiceIndex.value));

      const choice = options[settingsChoiceIndex.value - 2];
      if (!choice || !id) {
        uiMode.value = 'settings';
        return;
      }

      switch (id) {
        case 'units':
          settings.value.units = choice === '°F' ? 'F' : 'C';
          break;
        case 'pmDisplay':
          settings.value.pmDisplay = choice === 'USAQI' ? 'usAqi' : 'ugm3';
          break;
        case 'displayInterval':
          settings.value.displayInterval = choice as Settings['displayInterval'];
          if (choice === 'Display Off') {
            emit('update:activeMetric', 'none');
          }
          break;
        case 'pmInterval':
          settings.value.pmInterval = choice as Settings['pmInterval'];
          break;
        case 'otherSensorInterval':
          settings.value.otherSensorInterval = choice as Settings['otherSensorInterval'];
          break;
        case 'gpsMode':
          settings.value.gpsMode =
            choice === 'Always Off' ? 'off' : choice === 'Always On' ? 'always' : 'tracking';
          break;
        case 'connectionMode':
          applyConnectionMode(
            choice === 'Stationary'
              ? 'stationary'
              : choice === 'Portable'
                ? 'portable'
                : 'offline'
          );
          break;
        case 'autoLockSeconds':
          settings.value.autoLockSeconds =
            choice === 'Off'
              ? 0
              : choice === '30 Seconds'
                ? 30
                : choice === '60 Seconds'
                  ? 60
                  : 10;
          break;
      }

      uiMode.value = 'settings';
      return;
    }

    if (uiMode.value === 'about') {
      if (aboutIndex.value === 0) {
        closeMenus();
        return;
      }

      uiMode.value = 'menu';
      menuIndex.value = Math.max(
        0,
        menuItems.value.findIndex((item) => item.key === 'about')
      );
      return;
    }

    if (uiMode.value === 'confirm') {
      if (confirmIndex.value === 0) {
        closeMenus();
        return;
      }

      if (confirmIndex.value === 1) {
        uiMode.value = 'settings';
        return;
      }

      if (confirmIndex.value === 3) {
        uiMode.value = 'settings';
        return;
      }

      if (confirmIndex.value === 4) {
        trackingActive.value = false;
        lastSelectedTag.value = null;
        flashFeedback('Data cleared');
        closeMenus();
        return;
      }

      return;
    }

    if (uiMode.value === 'tag-list') {
      if (tagListIndex.value === 0) {
        closeMenus();
        return;
      }

      if (tagListIndex.value === 1) {
        uiMode.value = 'menu';
        menuIndex.value = 0;
        return;
      }

      const tags = sourceTags;
      const tag = tags[tagListIndex.value - 2];
      if (tag) {
        lastSelectedTag.value = tag;
        flashFeedback(`Tag '${tag}' saved`);
      }

      closeMenus();
    }
  };

  watch(
    () => settings.value.autoLockSeconds,
    () => {
      scheduleAutoLock();
    }
  );

  watch(deviceLocked, (locked) => {
    if (locked) {
      clearAutoLockTimeout();
      return;
    }

    scheduleAutoLock();
  });

  watch(
    () => props.powered,
    (powered) => {
      clearShutdownTimeout();

      if (powered) {
        powerPhase.value = 'on';
        emit('power-phase-change', 'on');
        deviceLocked.value = false;
        trackingActive.value = false;
        pressedButton.value = null;
        feedbackMessage.value = null;
        goToHomeScreen();
        scheduleAutoLock();
        return;
      }

      if (powerPhase.value === 'off') return;

      clearAutoLockTimeout();
      pressedButton.value = null;
      clearLockToggleTimeout();
      goToHomeScreen();
      deviceLocked.value = false;
      powerPhase.value = 'shutting-down';
      emit('power-phase-change', 'shutting-down');

      shutdownTimeout = setTimeout(() => {
        trackingActive.value = false;
        feedbackMessage.value = null;
        powerPhase.value = 'off';
        emit('power-phase-change', 'off');
        shutdownTimeout = null;
      }, 1400);
    },
    { immediate: true }
  );

  onBeforeUnmount(() => {
    if (feedbackTimeout) clearTimeout(feedbackTimeout);
    clearLockToggleTimeout();
    clearAutoLockTimeout();
    clearShutdownTimeout();
  });

  onMounted(() => {
    scheduleAutoLock();
  });

</script>

<style scoped>
  .ag-go-button {
    transition:
      transform 80ms ease,
      filter 80ms ease,
      opacity 80ms ease;
    outline: none;
    -webkit-tap-highlight-color: transparent;
    transform-box: fill-box;
    transform-origin: center;
  }

  .ag-go-button:focus,
  .ag-go-button:focus-visible {
    outline: none;
  }

  .ag-go-button:hover {
    filter: brightness(0.98);
  }

  .ag-go-button--pressed {
    transform: scale(0.94);
    filter: brightness(0.92);
  }
</style>
