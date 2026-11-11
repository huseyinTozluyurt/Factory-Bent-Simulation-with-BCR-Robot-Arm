import { useState } from 'react'
import { cycle } from '../data/content.js'

/* Small SVG helpers */

function Box({ x, y, w, h, title, sub, kind = 'ros' }) {
  return (
    <g className={`dg-box dg-${kind}`}>
      <rect x={x} y={y} width={w} height={h} rx="10" />
      <text x={x + w / 2} y={sub ? y + h / 2 - 4 : y + h / 2 + 5} textAnchor="middle" className="dg-title">
        {title}
      </text>
      {sub && (
        <text x={x + w / 2} y={y + h / 2 + 15} textAnchor="middle" className="dg-sub">
          {sub}
        </text>
      )}
    </g>
  )
}

function Ring({ cx, cy, r = 46, title, sub }) {
  return (
    <g className="dg-box dg-ring">
      <circle cx={cx} cy={cy} r={r} />
      <circle cx={cx} cy={cy} r={r - 12} className="dg-ring-inner" />
      <text x={cx} y={cy - 2} textAnchor="middle" className="dg-title">{title}</text>
      <text x={cx} y={cy + 16} textAnchor="middle" className="dg-sub">{sub}</text>
    </g>
  )
}

function Edge({ d, label, lx, ly, dashed }) {
  return (
    <g className="dg-edge">
      <path d={d} markerEnd="url(#arrow)" className={dashed ? 'dashed' : ''} />
      {label && (
        <text x={lx} y={ly} textAnchor="middle" className="dg-label">
          {label}
        </text>
      )}
    </g>
  )
}

function Defs() {
  return (
    <defs>
      <marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
        <path d="M0,0 L10,5 L0,10 z" className="dg-arrowhead" />
      </marker>
    </defs>
  )
}

/* ---------- ROS 2 architecture ---------- */

export function ArchitectureDiagram() {
  return (
    <div className="diagram">
      <svg viewBox="0 0 1200 560" role="img" aria-label="ROS 2 node architecture">
        <Defs />
        {/* Gazebo world frame */}
        <rect x="10" y="110" width="200" height="330" rx="14" className="dg-frame" />
        <text x="110" y="134" textAnchor="middle" className="dg-frame-label">GAZEBO HARMONIC</text>

        <Box x={20} y={20} w={190} h={60} title="order_gui" sub="PyQt5 window" kind="gui" />

        <Box x={30} y={150} w={160} h={56} title="Overhead camera" sub="13 m, top-down" kind="gz" />
        <Box x={30} y={250} w={160} h={56} title="Side camera" sub="belt height" kind="gz" />
        <Box x={30} y={350} w={160} h={56} title="Release camera" sub="above bent 2" kind="gz" />

        <Box x={330} y={150} w={200} h={56} title="ring_object_detector" sub="contours · minAreaRect" />
        <Box x={330} y={250} w={200} h={56} title="side_camera_checker" sub="MOG2 · Canny · height" />
        <Box x={330} y={350} w={200} h={56} title="release_zone_monitor" sub="empty-belt difference" />

        <Box x={700} y={215} w={200} h={126} title="vision_pick_place" sub="state machine + URDF IK" kind="core" />

        <Box x={962} y={150} w={220} h={56} title="BCR 7-axis arm" sub="joint trajectory controller" kind="gz" />
        <Box x={962} y={350} w={220} h={56} title="bcr_endpoint_object_attacher" sub="grasp · bents · spawning" />

        <Box x={330} y={480} w={850} h={56} title="Gazebo world: bents, products, bins" sub="moved through gz-transport set_pose / create" kind="gz" />

        <Edge d="M190,178 H322" label="ros_gz_bridge" lx={258} ly={170} />
        <Edge d="M190,278 H322" label="ros_gz_bridge" lx={258} ly={270} />
        <Edge d="M190,378 H322" label="ros_gz_bridge" lx={258} ly={370} />

        <Edge d="M530,178 C620,178 610,240 692,240" label="/vision/ring1_objects" lx={612} ly={168} />
        <Edge d="M530,278 H692" label="/vision/side_check" lx={611} ly={270} />
        <Edge d="M530,378 C620,378 610,318 692,318" label="/vision/release_zone" lx={612} ly={398} />

        <Edge d="M900,250 C935,250 930,178 954,178" label="joint_trajectory" lx={930} ly={140} />
        <Edge d="M900,306 C935,306 930,378 954,378" label="hold · attach · release" lx={930} ly={430} />

        <Edge d="M210,50 H1194 V378 H1190" label="/spawn_order  ·  /factory_settings" lx={700} ly={42} />
        <Edge d="M1070,406 V472" label="gz-transport" lx={1120} ly={445} />
      </svg>
      <div className="legend">
        <span><i className="sw sw-gz" /> Gazebo</span>
        <span><i className="sw sw-ros" /> ROS 2 node</span>
        <span><i className="sw sw-core" /> Controller</span>
        <span><i className="sw sw-gui" /> Operator GUI</span>
      </div>
    </div>
  )
}

/* ---------- Pipeline factory routing ---------- */

export function PipelineDiagram() {
  return (
    <div className="diagram">
      <svg viewBox="0 0 1200 470" role="img" aria-label="Pipeline factory routing between rings, lines and docks">
        <Defs />
        <Box x={10} y={190} w={110} h={46} title="S1 inbound" kind="line" />
        <Ring cx={240} cy={213} title="C1" sub="recyclable?" />

        <Box x={360} y={68} w={80} h={42} title="S2" kind="line" />
        <Ring cx={540} cy={89} title="C2" sub="cardboard?" />
        <Box x={710} y={26} w={150} h={42} title="Dock 1 · cardboard" kind="dock" />
        <Box x={710} y={110} w={150} h={42} title="Dock 2" kind="dock" />

        <Box x={360} y={309} w={80} h={42} title="S3" kind="line" />
        <Ring cx={540} cy={330} title="C3" sub="size?" />
        <Box x={710} y={232} w={150} h={42} title="Dock 3 · large" kind="dock" />
        <Box x={710} y={360} w={80} h={42} title="S6" kind="line" />
        <Ring cx={900} cy={381} title="C4" sub="shipping" />
        <Box x={1010} y={300} w={180} h={42} title="Dock 4 · express" kind="dock" />
        <Box x={1010} y={410} w={180} h={42} title="Dock 5 · standard" kind="dock" />

        <Edge d="M120,213 H186" label="A1 all" lx={153} ly={204} />
        <Edge d="M272,180 L354,100" label="A2 recyclable" lx={282} ly={128} />
        <Edge d="M272,246 L354,322" label="A3 rest" lx={290} ly={302} />
        <Edge d="M440,89 H486" label="A4" lx={463} ly={80} />
        <Edge d="M440,330 H486" label="A5" lx={463} ly={321} />
        <Edge d="M586,72 L704,50" label="A6 cardboard" lx={640} ly={50} />
        <Edge d="M586,106 L704,128" label="A7 rest" lx={640} ly={134} />
        <Edge d="M586,313 L704,258" label="A9 large" lx={636} ly={274} />
        <Edge d="M586,350 L704,378" label="A8 rest" lx={636} ly={384} />
        <Edge d="M790,381 H846" label="A10" lx={818} ly={372} />
        <Edge d="M944,365 L1004,325" label="A11 small" lx={952} ly={330} />
        <Edge d="M944,397 L1004,428" label="A12 rest" lx={956} ly={436} />
      </svg>
      <div className="legend">
        <span><i className="sw sw-line" /> Transport line</span>
        <span><i className="sw sw-ring" /> Sorting ring</span>
        <span><i className="sw sw-dock" /> Dock</span>
        <span>A1–A12 = BCR arms</span>
      </div>
    </div>
  )
}

/* ---------- Pick and place cycle stepper ---------- */

export function CycleStepper() {
  const [i, setI] = useState(0)
  const step = cycle[i]
  return (
    <div className="stepper">
      <ol className="stepper-track">
        {cycle.map((s, k) => (
          <li key={s.id}>
            <button
              type="button"
              className={k === i ? 'active' : k < i ? 'past' : ''}
              onClick={() => setI(k)}
            >
              <span className="num">{k + 1}</span>
              {s.id}
            </button>
          </li>
        ))}
      </ol>
      <div className="stepper-body">
        <p className="stepper-title">
          Step {i + 1} · {step.id}
        </p>
        <p>{step.text}</p>
        <div className="stepper-nav">
          <button type="button" disabled={i === 0} onClick={() => setI(i - 1)}>← Previous</button>
          <button type="button" disabled={i === cycle.length - 1} onClick={() => setI(i + 1)}>Next →</button>
        </div>
      </div>
    </div>
  )
}
