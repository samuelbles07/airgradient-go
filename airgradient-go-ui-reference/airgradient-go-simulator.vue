<template>
  <section class="section">
    <div class="container py-5">
      <div class="row align-items-start g-4">
        <div class="col-12 col-lg-5">
          <h1 class="mb-3">AirGradient Go Simulator</h1>
          <p>
            We built this simulator to test in detail the UI, and be able to get feedback from our community. If you have any feedback, please use this <a href="https://forms.gle/ftj1b3vbzBoHxXwN7" target="_blank">form</a>.
          </p>
          <p>
            If you want to be informed when the actual device will become available, you can signup <a href="https://forms.gle/ftj1b3vbzBoHxXwN7" target="_blank">on the same form</a>.
          </p>
          <p>Have Fun!</p>
        </div>

        <div class="col-12 col-lg-7 d-flex justify-content-center">
          <div class="simulator-panel">
            <AirGradientGoDeviceSvg
              v-model:activeMetric="activeMetric"
              :powered="powered"
              class="ag-go-device"
              @power-phase-change="powerPhase = $event"
            />
            <button class="power-button" type="button" :disabled="powerPhase === 'shutting-down'" @click="togglePower">
              {{ powerPhase === 'off' ? 'Turn On' : powerPhase === 'shutting-down' ? 'Powering Off...' : 'Turn Off' }}
            </button>
          </div>
        </div>
      </div>
    </div>
  </section>
</template>

<script setup lang="ts">
  import { useHead } from '#imports';
  import { ref } from 'vue';

  const activeMetric = ref<
    'none' | 'co2' | 'pm25' | 'temp' | 'humidity' | 'tvoc' | 'nox' | 'pressure' | 'altitude'
  >('none');
  const powered = ref(true);
  const powerPhase = ref<'on' | 'shutting-down' | 'off'>('on');

  const togglePower = () => {
    if (powerPhase.value === 'shutting-down') return;
    powered.value = powerPhase.value === 'off';
  };

  useHead({
    title: 'AirGradient Go Simulator',
    meta: [
      {
        name: 'description',
        content: 'Online UI simulator for the AirGradient Go portable air quality monitor.'
      }
    ]
  });
</script>

<style scoped>
  .simulator-panel {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 1rem;
  }

  .ag-go-device {
    width: min(380px, 100%);
    height: auto;
  }

  .power-button {
    min-width: 9rem;
    border: 0;
    border-radius: 999px;
    padding: 0.7rem 1.1rem;
    background: #1f2a2e;
    color: #fff;
    font-weight: 700;
    letter-spacing: 0.02em;
    transition:
      transform 120ms ease,
      opacity 120ms ease,
      background-color 120ms ease;
  }

  .power-button:hover:not(:disabled) {
    background: #12191c;
  }

  .power-button:active:not(:disabled) {
    transform: translateY(1px);
  }

  .power-button:disabled {
    opacity: 0.65;
    cursor: wait;
  }
</style>
