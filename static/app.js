const trackSelect = document.getElementById('track');
const spacingInput = document.getElementById('spacing');
const solveButton = document.getElementById('solve');
const spinnerEl = document.getElementById('spinner');
const statusEl = document.getElementById('status');
const statsEl = document.getElementById('stats');
const mainEl = document.querySelector('main');

const trackSvg = d3.select('#track-view');
const speedSvg = d3.select('#speed-trace');

const tooltip = d3.select('body').append('div').attr('class', 'tooltip');

let trackInfoBySlug = {};

// Real circuit names for display -- the underlying <option value> stays the data/ slug the
// API expects. Falls back to a capitalized slug for any track not listed here, so a newly
// exported track still shows up (just less prettily) with no code change required.
const TRACK_NAMES = {
  bahrain: 'Bahrain International Circuit',
  baku: 'Baku City Circuit',
  catalunya: 'Circuit de Barcelona-Catalunya',
  cota: 'Circuit of the Americas',
  hockenheim: 'Hockenheimring',
  hungaroring: 'Hungaroring',
  interlagos: 'Interlagos',
  monaco: 'Circuit de Monaco',
  montreal: 'Circuit Gilles Villeneuve',
  monza: 'Autodromo Nazionale Monza',
  paulricard: 'Circuit Paul Ricard',
  redbullring: 'Red Bull Ring',
  shanghai: 'Shanghai International Circuit',
  silverstone: 'Silverstone Circuit',
  singapore: 'Marina Bay Street Circuit',
  spa: 'Spa-Francorchamps',
  suzuka: 'Suzuka Circuit',
  yasmarina: 'Yas Marina Circuit',
};

function displayName(slug) {
  return TRACK_NAMES[slug] || (slug.charAt(0).toUpperCase() + slug.slice(1));
}

const STORAGE_KEY_TRACK = 'f1sim.track';
const STORAGE_KEY_SPACING = 'f1sim.spacing';

function restorePreferences() {
  try {
    const savedTrack = localStorage.getItem(STORAGE_KEY_TRACK);
    const savedSpacing = localStorage.getItem(STORAGE_KEY_SPACING);
    if (savedTrack && trackInfoBySlug[savedTrack]) trackSelect.value = savedTrack;
    if (savedSpacing) spacingInput.value = savedSpacing;
  } catch (err) {
    // localStorage unavailable (e.g. private browsing) -- just skip restoring.
  }
}

function savePreferences(track, spacing) {
  try {
    localStorage.setItem(STORAGE_KEY_TRACK, track);
    localStorage.setItem(STORAGE_KEY_SPACING, String(spacing));
  } catch (err) {
    // ignore
  }
}

async function loadTracks() {
  const res = await fetch('/api/tracks');
  const tracks = await res.json();
  trackInfoBySlug = Object.fromEntries(tracks.map(t => [t.slug, t]));
  trackSelect.innerHTML = tracks.map(t => {
    const name = displayName(t.slug);
    const label = t.known_issue ? `${name} ⚠️` : name;
    const title = t.known_issue ? ` title="${t.known_issue.replace(/"/g, '&quot;')}"` : '';
    return `<option value="${t.slug}"${title}>${label}</option>`;
  }).join('');
}

function formatLapTime(seconds) {
  const m = Math.floor(seconds / 60);
  const s = (seconds % 60).toFixed(3).padStart(6, '0');
  return `${m}:${s}`;
}

function renderTrack(result) {
  trackSvg.selectAll('*').remove();
  const width = trackSvg.node().clientWidth;
  const height = trackSvg.node().clientHeight;

  const xExtent = d3.extent(result.x_ref);
  const yExtent = d3.extent(result.y_ref);
  const padding = 40;
  const scale = Math.min(
    (width - 2 * padding) / (xExtent[1] - xExtent[0]),
    (height - 2 * padding) / (yExtent[1] - yExtent[0]),
  );
  const xCenter = (xExtent[0] + xExtent[1]) / 2;
  const yCenter = (yExtent[0] + yExtent[1]) / 2;

  const xScale = x => width / 2 + (x - xCenter) * scale;
  // SVG y grows downward; flip so track northing grows upward on screen.
  const yScale = y => height / 2 - (y - yCenter) * scale;

  const g = trackSvg.append('g');
  const boundaryLine = d3.line().x(d => xScale(d[0])).y(d => yScale(d[1]));

  g.append('path').datum(d3.zip(result.x_left, result.y_left)).attr('class', 'boundary').attr('d', boundaryLine);
  g.append('path').datum(d3.zip(result.x_right, result.y_right)).attr('class', 'boundary').attr('d', boundaryLine);
  g.append('path').datum(d3.zip(result.x_ref, result.y_ref)).attr('class', 'reference').attr('d', boundaryLine);

  const colorScale = d3.scaleSequential(d3.interpolateTurbo).domain(d3.extent(result.speed_kmh));
  const n = result.x_line.length;
  const segments = g.append('g');
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    segments.append('line')
      .attr('x1', xScale(result.x_line[i]))
      .attr('y1', yScale(result.y_line[i]))
      .attr('x2', xScale(result.x_line[j]))
      .attr('y2', yScale(result.y_line[j]))
      .attr('stroke', colorScale(result.speed_kmh[i]))
      .attr('stroke-width', 3)
      .on('mousemove', (event) => {
        tooltip.style('opacity', 1)
          .html(`s = ${result.s[i].toFixed(0)} m<br>speed = ${result.speed_kmh[i].toFixed(1)} km/h<br>n = ${result.n_profile[i].toFixed(2)} m`)
          .style('left', (event.pageX + 12) + 'px')
          .style('top', (event.pageY - 12) + 'px');
      })
      .on('mouseleave', () => tooltip.style('opacity', 0));
  }

  // Start/finish line: the segment across the track at index 0 (s=0), same point the lap
  // time is measured from and back to.
  const sf = g.append('g');
  sf.append('line')
    .attr('class', 'sf-line')
    .attr('x1', xScale(result.x_left[0])).attr('y1', yScale(result.y_left[0]))
    .attr('x2', xScale(result.x_right[0])).attr('y2', yScale(result.y_right[0]));
  sf.append('text')
    .attr('class', 'sf-label')
    .attr('text-anchor', 'middle')
    .attr('x', xScale(result.x_ref[0]))
    .attr('y', yScale(result.y_ref[0]) - 10)
    .text('S/F');

  const zoom = d3.zoom().scaleExtent([0.5, 20]).on('zoom', (event) => {
    g.attr('transform', event.transform);
  });
  trackSvg.call(zoom);

  // Fixed overlay (outside the zoomable/pannable `g`) so it stays put while exploring the map.
  renderSpeedLegend(trackSvg, width, height, colorScale, d3.extent(result.speed_kmh));
}

function renderSpeedLegend(svg, width, height, colorScale, speedExtent) {
  const legendWidth = 160;
  const legendHeight = 10;
  const x0 = 20;
  const y0 = height - 32;

  const gradientId = 'speed-gradient';
  svg.select(`#${gradientId}`).remove();
  const gradient = svg.append('defs').append('linearGradient')
    .attr('id', gradientId).attr('x1', '0%').attr('x2', '100%').attr('y1', '0%').attr('y2', '0%');
  const steps = 10;
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    gradient.append('stop')
      .attr('offset', `${t * 100}%`)
      .attr('stop-color', colorScale(speedExtent[0] + t * (speedExtent[1] - speedExtent[0])));
  }

  const legend = svg.append('g');
  legend.append('text')
    .attr('class', 'axis-label')
    .attr('x', x0)
    .attr('y', y0 - 8)
    .text('Speed (km/h)');
  legend.append('rect')
    .attr('class', 'legend-bar')
    .attr('x', x0).attr('y', y0)
    .attr('width', legendWidth).attr('height', legendHeight)
    .attr('rx', 2)
    .attr('fill', `url(#${gradientId})`);
  legend.append('text')
    .attr('class', 'axis-label')
    .attr('x', x0)
    .attr('y', y0 + legendHeight + 14)
    .text(Math.round(speedExtent[0]));
  legend.append('text')
    .attr('class', 'axis-label')
    .attr('text-anchor', 'end')
    .attr('x', x0 + legendWidth)
    .attr('y', y0 + legendHeight + 14)
    .text(Math.round(speedExtent[1]));
}

function renderSpeedTrace(result) {
  speedSvg.selectAll('*').remove();
  const width = speedSvg.node().clientWidth;
  const height = speedSvg.node().clientHeight;
  const margin = { top: 16, right: 16, bottom: 44, left: 56 };

  const x = d3.scaleLinear().domain(d3.extent(result.s)).range([margin.left, width - margin.right]);
  const y = d3.scaleLinear().domain([0, d3.max(result.speed_kmh) * 1.05]).range([height - margin.bottom, margin.top]);

  const g = speedSvg.append('g');
  g.append('g').attr('class', 'axis').attr('transform', `translate(0,${height - margin.bottom})`).call(d3.axisBottom(x).ticks(6));
  g.append('g').attr('class', 'axis').attr('transform', `translate(${margin.left},0)`).call(d3.axisLeft(y).ticks(5));

  g.append('text')
    .attr('class', 'axis-label')
    .attr('text-anchor', 'middle')
    .attr('x', margin.left + (width - margin.left - margin.right) / 2)
    .attr('y', height - 6)
    .text('Distance around lap (m)');

  g.append('text')
    .attr('class', 'axis-label')
    .attr('text-anchor', 'middle')
    .attr('transform', 'rotate(-90)')
    .attr('x', -(margin.top + (height - margin.top - margin.bottom) / 2))
    .attr('y', 14)
    .text('Speed (km/h)');

  const line = d3.line().x((d, i) => x(result.s[i])).y(d => y(d));
  g.append('path')
    .datum(result.speed_kmh)
    .attr('fill', 'none')
    .attr('stroke', '#1f77b4')
    .attr('stroke-width', 1.5)
    .attr('d', line);

  const crosshair = g.append('line').attr('class', 'crosshair')
    .attr('y1', margin.top).attr('y2', height - margin.bottom).style('opacity', 0);

  speedSvg.on('mousemove', (event) => {
    const [mx] = d3.pointer(event);
    const sVal = x.invert(mx);
    const i = d3.bisector(d => d).left(result.s, sVal);
    if (i < 0 || i >= result.s.length) return;
    crosshair.attr('x1', x(result.s[i])).attr('x2', x(result.s[i])).style('opacity', 1);
    tooltip.style('opacity', 1)
      .html(`s = ${result.s[i].toFixed(0)} m<br>speed = ${result.speed_kmh[i].toFixed(1)} km/h`)
      .style('left', (event.pageX + 12) + 'px')
      .style('top', (event.pageY - 12) + 'px');
  }).on('mouseleave', () => {
    crosshair.style('opacity', 0);
    tooltip.style('opacity', 0);
  });
}

function renderStats(result) {
  const real = result.real_lap_time_2018_s;
  const realRow = real == null
    ? ''
    : `
    <div class="stat"><span class="label">2018 pole time</span><span class="value">${formatLapTime(real)}</span></div>
    <div class="stat"><span class="label">Gap to 2018 pole</span><span class="value">${(result.lap_time_s - real >= 0 ? '+' : '')}${(result.lap_time_s - real).toFixed(3)} s</span></div>`;

  const issue = trackInfoBySlug[result.track] && trackInfoBySlug[result.track].known_issue;
  const warningBanner = issue ? `<div class="warning">⚠️ ${issue}</div>` : '';

  statsEl.innerHTML = `${warningBanner}
    <div class="stat"><span class="label">Solver lap time</span><span class="value highlight">${formatLapTime(result.lap_time_s)}</span></div>${realRow}
    <div class="stat"><span class="label">Avg. line offset</span><span class="value">${result.mean_abs_n.toFixed(2)} m</span></div>
    <div class="stat"><span class="label">Max. line offset</span><span class="value">${result.max_abs_n.toFixed(2)} m</span></div>
  `;
}

function renderError(message) {
  statsEl.innerHTML = `<div class="error">${message}</div>`;
  trackSvg.selectAll('*').remove();
  speedSvg.selectAll('*').remove();
}

async function solve() {
  const track = trackSelect.value;
  const spacing = parseFloat(spacingInput.value) || 25;
  savePreferences(track, spacing);
  statusEl.textContent = 'Solving…';
  spinnerEl.hidden = false;
  solveButton.disabled = true;
  mainEl.classList.add('loading');
  try {
    const res = await fetch(`/api/solve?track=${encodeURIComponent(track)}&spacing=${spacing}`);
    if (!res.ok) {
      renderError(await res.text());
      statusEl.textContent = 'Failed';
      return;
    }
    const result = await res.json();
    renderTrack(result);
    renderSpeedTrace(result);
    renderStats(result);
    statusEl.textContent = '';
  } catch (err) {
    renderError(String(err));
    statusEl.textContent = 'Failed';
  } finally {
    solveButton.disabled = false;
    spinnerEl.hidden = true;
    mainEl.classList.remove('loading');
  }
}

solveButton.addEventListener('click', solve);
loadTracks().then(restorePreferences);
