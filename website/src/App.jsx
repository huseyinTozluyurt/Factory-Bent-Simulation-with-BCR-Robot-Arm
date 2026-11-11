import { useState } from 'react'
import {
  nav, stats, tech, features, nodes, worlds, cameras, motionStyles, labs, labApi,
  pipelineTests, eras, phases, reports, roadmap, img, report, REPO_URL,
} from './data/content.js'
import { LightboxProvider, Figure, Section, Code, Compare, useReveal, useActiveSection } from './components/ui.jsx'
import { ArchitectureDiagram, PipelineDiagram, CycleStepper } from './components/diagrams.jsx'

const navIds = nav.map((n) => n.id)

function Header() {
  const active = useActiveSection(navIds)
  const [open, setOpen] = useState(false)
  return (
    <header className="topbar">
      <div className="container topbar-inner">
        <a href="#top" className="brand" onClick={() => setOpen(false)}>
          <span className="brand-mark">FB</span>
          <span>Factory Bent<span className="brand-dim"> · BCR Arm</span></span>
        </a>
        <button className="menu-btn" aria-label="Menu" aria-expanded={open} onClick={() => setOpen(!open)}>
          {open ? '✕' : '☰'}
        </button>
        <nav className={open ? 'open' : ''}>
          {nav.map((n) => (
            <a key={n.id} href={`#${n.id}`} className={active === n.id ? 'active' : ''} onClick={() => setOpen(false)}>
              {n.label}
            </a>
          ))}
        </nav>
      </div>
    </header>
  )
}

function Hero() {
  return (
    <section className="hero" id="top">
      <div className="hero-bg" style={{ backgroundImage: `url(${img('dual_world_decor.png')})` }} />
      <div className="container hero-inner">
        <div className="hero-text">
          <p className="kicker">ROS 2 · Gazebo · OpenCV · Robotics</p>
          <h1>
            Factory Bent Simulation <span className="accent">with a BCR Robot Arm</span>
          </h1>
          <p className="hero-lead">
            A 7-axis Black Coffee Robotics arm watches moving products through three cameras, picks each one off a
            conveyor bent and places it on another bent, or drops it into a trash or recycling bin. Every decision
            comes from OpenCV.
          </p>
          <div className="hero-cta">
            <a className="btn primary" href="#overview">Explore the project</a>
            <a className="btn" href="#run">Run it yourself</a>
          </div>
          <ul className="chips">
            {tech.map((t) => <li key={t}>{t}</li>)}
          </ul>
        </div>
        <div className="hero-shot">
          <Figure name="screenshot_2026-10-31_order_gui_dual_detection.png" caption="Order GUI and the live OpenCV detection window in the dual circular world" />
        </div>
      </div>
      <div className="container">
        <ul className="stats">
          {stats.map((s) => (
            <li key={s.label}>
              <strong>{s.value}</strong>
              <span>{s.label}</span>
            </li>
          ))}
        </ul>
      </div>
    </section>
  )
}

function Overview() {
  return (
    <Section
      id="overview"
      kicker="01 · Overview"
      title="From a camera image to a robot moving a product"
      lead="The project is a simulated factory cell. Products ride on conveyor belts (“bents”). Cameras look at them, OpenCV finds and classifies them, and a BCR arm picks them up and moves them where they belong. Around that core sit a product generator, an operator GUI, ten worlds and a lab for testing new detectors."
    >
      <div className="grid features reveal">
        {features.map((f) => (
          <article key={f.title} className="card feature">
            <span className="feature-icon" aria-hidden>{f.icon}</span>
            <h3>{f.title}</h3>
            <p>{f.text}</p>
          </article>
        ))}
      </div>
      <div className="split reveal">
        <Figure name="screenshot_2026-10-31_dual_circular_factory.png" caption="The dual circular cell in Gazebo: two bents, the BCR arm in the middle, the trash bin and the operator desk" />
        <div className="prose">
          <h3>Goals</h3>
          <ul className="ticks">
            <li>Detect products on a moving belt using classic OpenCV and, optionally, YOLO.</li>
            <li>Turn pixel detections into world coordinates the arm can reach.</li>
            <li>Pick and place from one bent to another, closed-loop, while both belts move.</li>
            <li>Generate products for the singular factory bent and every other world.</li>
            <li>Make it easy to swap in a new detection algorithm and measure it.</li>
          </ul>
          <p className="muted">
            Everything runs on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic, and starts with one command.
          </p>
        </div>
      </div>
    </Section>
  )
}

function Architecture() {
  return (
    <Section
      id="architecture"
      alt
      kicker="02 · Architecture"
      title="Six ROS 2 nodes around one state machine"
      lead="Cameras in Gazebo are bridged into ROS. Three vision nodes each answer one question, and the controller only moves the arm when their answers agree."
    >
      <div className="reveal"><ArchitectureDiagram /></div>
      <div className="grid nodes reveal">
        {nodes.map((n) => (
          <article key={n.name} className="card node">
            <code>{n.name}</code>
            <p>{n.role}</p>
          </article>
        ))}
      </div>
      <h3 className="sub-head reveal">One pick-and-place cycle</h3>
      <p className="muted reveal">Click through the steps the controller goes through for every product.</p>
      <div className="reveal"><CycleStepper /></div>
    </Section>
  )
}

function Worlds() {
  const gallery = worlds.filter((w) => w.image)
  const [sel, setSel] = useState(gallery[0].key)
  const w = gallery.find((x) => x.key === sel)
  return (
    <Section
      id="worlds"
      kicker="03 · Worlds"
      title="Ten .sdf worlds, one shared cell"
      lead="Every single-cell world is generated from the same layout. The arm, the catch station at (-2.58, 0), the place point at (2.22, -0.37), the cameras and the bins never move; only the belts change. A WORLD_LAYOUTS table tells every node the belt geometry."
    >
      <div className="tabs reveal" role="tablist">
        {gallery.map((g) => (
          <button key={g.key} role="tab" aria-selected={sel === g.key} className={sel === g.key ? 'active' : ''} onClick={() => setSel(g.key)}>
            {g.name}
          </button>
        ))}
      </div>
      <div className="world fade-in" key={w.key}>
        <Figure name={w.image} caption={`${w.name} world`} />
        <div className="world-info">
          <h3>{w.name}</h3>
          <p>{w.text}</p>
          <dl>
            <dt>Bent 1</dt><dd>{w.bent1}</dd>
            <dt>Bent 2</dt><dd>{w.bent2}</dd>
            <dt>Arm delivers to</dt><dd>{w.delivers}</dd>
          </dl>
          <Code>{w.cmd}</Code>
        </div>
      </div>

      <div className="table-wrap reveal">
        <table>
          <thead>
            <tr><th>Launcher</th><th>Bent 1</th><th>Bent 2</th><th>The arm delivers to</th></tr>
          </thead>
          <tbody>
            {worlds.map((x) => (
              <tr key={x.key}><td><code>{x.cmd}</code></td><td>{x.bent1}</td><td>{x.bent2}</td><td>{x.delivers}</td></tr>
            ))}
            <tr><td><code>./run_pipeline_factory.sh</code></td><td colSpan={2}>4 rings and 9 lines</td><td>12 arms and 5 docks</td></tr>
          </tbody>
        </table>
      </div>

      <div className="split reveal">
        <Figure name="camera_views_before_after_decor.png" caption="Camera views before (left) and after (right) the factory decor was added: the detectors still see the same belt" />
        <div className="prose">
          <h3>A believable factory, without breaking vision</h3>
          <p>
            The decor pass added a hall, safety fences, hazard-striped floor markings, stack lights, racks and signs.
            The overhead and side camera views were checked before and after so the detectors still see clean belts.
          </p>
          <p className="muted">
            On a slow PC, start the plain variant <code>dual_circular_factory_conveyor_plain.sdf</code>, which has no decor.
          </p>
        </div>
      </div>
    </Section>
  )
}

function Vision() {
  return (
    <Section
      id="vision"
      alt
      kicker="04 · Vision pipeline"
      title="Three cameras, three questions"
      lead="Each camera has one job and its own OpenCV pipeline. The arm only grabs a product when the overhead track and the side-camera blob agree within 0.20 m; otherwise the product rides on."
    >
      <div className="grid cams reveal">
        {cameras.map((c) => (
          <article key={c.name} className="card cam">
            <Figure name={c.image} caption={`${c.name} camera`} />
            <div className="cam-body">
              <h3>{c.name} camera</h3>
              <p className="muted small">{c.where}</p>
              <ol className="pipeline-steps">
                {c.steps.map((s) => <li key={s}>{s}</li>)}
              </ol>
              <p className="question">“{c.question}”</p>
            </div>
          </article>
        ))}
      </div>

      <div className="split reveal">
        <Compare before="overhead_raw.png" after="overhead_detections.png" beforeLabel="Raw overhead image" afterLabel="Detections" />
        <div className="prose">
          <h3>From pixels to metres</h3>
          <p>
            The overhead detector finds contours in each belt lane and fits a <code>minAreaRect</code> to every product.
            Each pixel is turned into a ray from the camera and intersected with the plane of the product’s top, so a tall
            box and a flat pouch both land at the right world position.
          </p>
          <p>
            The arm itself is masked out of the image using TF, and a ROI tracker predicts when a product will enter the
            catch station, including the arm’s reaction time.
          </p>
          <p className="muted">
            Drag the slider to compare the raw camera frame with the annotated detections.
          </p>
        </div>
      </div>

      <Figure className="wide reveal" name="screenshot_2026-10-30_ring_detection_live.png" caption="The live detection window: the ROI tracker locks on a product approaching the station, with Canny and grayscale debug views" />
      <p className="note reveal">
        The geometry modules (<code>vision_geometry.py</code>, <code>side_vision.py</code>, <code>release_zone.py</code>) don’t import ROS,
        so they can be tested offline on synthetic images.
      </p>
    </Section>
  )
}

function Arm() {
  return (
    <Section
      id="arm"
      kicker="05 · Pick and place"
      title="A closed-loop grasp on a moving belt"
      lead="The controller solves inverse kinematics from the robot’s own URDF, descends with feedback, and chooses how to carry each product from its measured height."
    >
      <div className="grid three reveal">
        <article className="card">
          <h3>🧮 URDF inverse kinematics</h3>
          <p>
            <code>urdf_ik.py</code> reads the kinematic chain from <code>/robot_description</code> and solves for the gripper tip
            with damped least squares. Solutions are pulled toward a taught pose, and postures that would hit a bin wall are rejected.
          </p>
        </article>
        <article className="card">
          <h3>🤏 Precise grasp</h3>
          <p>
            Fast descent to 10 cm above the product, then slow. The grip happens only when forward kinematics puts the tip
            within 1.5 cm of the top. The held product is pinned, so the gripper can’t push it away.
          </p>
        </article>
        <article className="card">
          <h3>🗑️ Bins</h3>
          <p>
            RECYCLE (green) sits at (0, +2.2) and TRASH (grey) at (0, −2.2). The arm travels high over the bin centre,
            drops the product, and its fall into the bin is simulated.
          </p>
        </article>
      </div>

      <h3 className="sub-head reveal">Six movement styles</h3>
      <div className="table-wrap reveal">
        <table>
          <thead><tr><th>Style</th><th>Lift</th><th>Swing</th><th>When <code>auto</code> uses it</th></tr></thead>
          <tbody>
            {motionStyles.map((m) => (
              <tr key={m.style}><td><code>{m.style}</code></td><td>{m.lift}</td><td>{m.swing}</td><td>{m.auto}</td></tr>
            ))}
          </tbody>
        </table>
      </div>

      <div className="grid two reveal">
        <Figure name="trash_recycle_bins.png" caption="Trash and recycling bins on either side of the arm" />
        <Figure name="order_gui_arm_movement.png" caption="Choosing the arm movement style in the order GUI" />
      </div>
    </Section>
  )
}

function Products() {
  return (
    <Section
      id="products"
      alt
      kicker="06 · Product generator"
      title="100 realistic products, plus your own"
      lead="The object generator puts orders of products on any bent, including the singular straight bent. Each type has a detailed look but a simple collision shape, and its top keeps the catalogue colour so the detectors still recognise it."
    >
      <Figure className="wide reveal" name="products_100_realistic.png" caption="The 100 built-in product types: 66 boxes, 26 cylinders and 8 spheres, with caps, lids, ribbons and fictional printed labels" />

      <div className="grid three reveal">
        <article className="card stat-card"><strong>66</strong><span>boxes</span></article>
        <article className="card stat-card"><strong>26</strong><span>cylinders</span></article>
        <article className="card stat-card"><strong>8</strong><span>spheres</span></article>
      </div>

      <div className="split reveal">
        <Figure name="order_gui_dark_theme_bins.png" caption="Order GUI: search, drag-to-sort, “Goes to” badges and simulation settings" />
        <div className="prose">
          <h3>The order GUI</h3>
          <p>
            Pick products from a picture grid, set where each type goes (bent 2, trash or recycling), the spacing, belt speed,
            simulation speed and run time, then press <em>Start simulation with this order</em>. After the world starts, the
            window comes back for live orders.
          </p>
          <h3>Spawn from a terminal</h3>
          <Code>{`ros2 topic pub --once /spawn_order std_msgs/msg/String \\
  "{data: '{\\"bent\\": 1, \\"items\\": [\\"paint_can\\", \\"cereal_box\\"], \\"spacing\\": 1.0, \\"start_offset\\": 1.5, \\"clear\\": true}'}"`}</Code>
        </div>
      </div>

      <div className="split reverse reveal">
        <Figure name="add_custom_product_dialog.png" caption="Add your own product from a 3D model or a photo" contain />
        <div className="prose">
          <h3>Bring your own product</h3>
          <p>
            Use <em>Add your own product…</em> in the GUI, or the command line, to turn an <code>.stl</code>, <code>.obj</code> or{' '}
            <code>.dae</code> model into a product. Products can be shared as a <code>.zip</code>.
          </p>
          <Code>python3 -m factory_bringup.custom_products add --name coffee_mug --mesh ~/mug.stl --size 0.12 0.09 0.10</Code>
        </div>
      </div>
    </Section>
  )
}

function Pipeline() {
  return (
    <Section
      id="pipeline"
      kicker="07 · Pipeline factory"
      title="Scaling one cell to 12 arms"
      lead="The pipeline factory has 4 sorting rings (C1–C4), 9 transport lines (S1–S9), 12 BCR arms (A1–A12) and 5 docks. Straight lines carry products between stations; on each ring, arms pick off the products that match their rule. The whole factory is generated from pipeline_layout.py."
    >
      <div className="reveal"><PipelineDiagram /></div>
      <div className="grid two reveal">
        <Figure name="screenshot_2026-10-31_pipeline_factory_overview.png" caption="Pipeline factory overview in Gazebo" />
        <Figure name="screenshot_2026-10-31_pipeline_factory_hall.png" caption="Inside the factory hall" />
        <Figure name="pipeline_top.png" caption="Top view of the generated layout" />
        <Figure name="pipeline_c1_station.png" caption="A station on sorting ring C1" />
      </div>
    </Section>
  )
}

function Lab() {
  return (
    <Section
      id="lab"
      alt
      kicker="08 · Vision lab"
      title="Plug in a detector, see it drive a robot"
      lead="You write a detector and one main(lab) function. The lab provides the camera, coordinate conversion, controller, arm, belts, bins and scoring in any world, with or without Gazebo."
    >
      <div className="split reveal">
        <div className="prose">
          <Code lang="python">{`# labs/lab_my_sorter.py
DETECTOR = "opencv_color"   # opencv_contours | opencv_color | opencv_bgsub | yolo | your class

def main(lab):
    lab.run({"glass_jar": "recycling", "pizza_box": "trash", "*": "bent2"}, count=10)
    print(lab.report())     # where each product went, and whether the label was right`}</Code>
          <Code>{`./run_vision_lab.sh sim --world dual                    # terminal 1: the simulation
./run_vision_lab.sh run my_sorter                       # terminal 2: your lab
./run_vision_lab.sh offline --synthetic 20 --world all  # score detectors without Gazebo
./run_vision_lab.sh run 10_pipeline_tests --fake        # pipeline tests, no Gazebo`}</Code>
        </div>
        <Figure name="vision_lab_offline_detectors.jpg" caption="Offline scoring of the detectors on synthetic images" contain />
      </div>

      <h3 className="sub-head reveal">The lab API</h3>
      <div className="table-wrap reveal">
        <table>
          <thead><tr><th>Call</th><th>Does</th></tr></thead>
          <tbody>{labApi.map(([c, d]) => <tr key={c}><td><code>{c}</code></td><td>{d}</td></tr>)}</tbody>
        </table>
      </div>

      <h3 className="sub-head reveal">Ten ready-made labs</h3>
      <ol className="labs reveal">
        {labs.map(([n, t, d]) => (
          <li key={n} className="card">
            <span className="lab-n">{n}</span>
            <div><strong>{t}</strong><p>{d}</p></div>
          </li>
        ))}
      </ol>

      <h3 className="sub-head reveal">Pipeline demo, act 1 and 2</h3>
      <div className="grid two reveal">
        <Figure name="pipeline_demo_detection_A1.jpg" caption="Act 1: opencv_contours labels products arriving at station A1" />
        <Figure name="pipeline_demo_after_A1.jpg" caption="Act 2: after arm A1 has sorted them" />
      </div>

      <h3 className="sub-head reveal">Pipeline test results</h3>
      <div className="table-wrap reveal">
        <table>
          <thead><tr><th>Case</th><th>Route</th><th>Result</th><th>Time</th></tr></thead>
          <tbody>
            {pipelineTests.map(([c, r, res, t]) => (
              <tr key={c}><td><code>{c}</code></td><td>{r}</td><td><span className="pass">{res}</span></td><td>{t}</td></tr>
            ))}
          </tbody>
        </table>
      </div>
      <div className="callout reveal">
        <strong>7 of 7 cases passed</strong>, but the detector’s labels were right in 48 of 60 decisions (80 %). Most misses were{' '}
        <code>shoe_box</code> labelled as <code>cookie_tin</code>, two types with similar top colours. That is exactly the kind of
        finding the lab is built to surface, and a good first target for a trained YOLO model.
      </div>
    </Section>
  )
}

function Timeline() {
  const [era, setEra] = useState(0)
  const e = eras[era]
  return (
    <Section
      id="timeline"
      kicker="09 · Evolution"
      title="From one belt to a 12-arm factory"
      lead="The project started in March 2026 as a single straight conveyor with an overhead camera. Between Oct 29 and Nov 1, 2026 it grew into the full system through about 27 phases, each traceable through a .bak_before_* backup."
    >
      <div className="eras reveal">
        {eras.map((x, k) => (
          <button key={x.n} className={k === era ? 'active' : ''} onClick={() => setEra(k)}>
            <span className="era-n">{x.n}</span>
            <span className="era-t">{x.title}</span>
            <span className="era-w">{x.when}</span>
          </button>
        ))}
      </div>
      <div className="era-body fade-in" key={e.n}>
        <p className="lead">{e.text}</p>
        <div className="grid two">
          {e.images.map(([n, c]) => <Figure key={n} name={n} caption={c} />)}
        </div>
      </div>

      <h3 className="sub-head reveal">Phase by phase</h3>
      <ol className="timeline reveal">
        {phases.map(([when, title, text]) => (
          <li key={when + title}>
            <time>{when}</time>
            <div>
              <strong>{title}</strong>
              <p>{text}</p>
            </div>
          </li>
        ))}
      </ol>

      <div className="grid two reveal">
        <Figure name="2026-03_straight_conveyor_first_version.png" caption="March 2026: the start" />
        <Figure name="dual_world_decor.png" caption="November 2026: the dual circular factory" />
      </div>
    </Section>
  )
}

function Run() {
  return (
    <Section
      id="run"
      alt
      kicker="10 · Run it"
      title="Start the whole factory with one command"
      lead="Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and the BCR arm packages in ~/ros2_ws/src."
    >
      <div className="grid two reveal">
        <div>
          <h3>1 · Install and build</h3>
          <Code>{`sudo apt install python3-opencv python3-pyqt5 python3-gz-transport13 python3-gz-msgs10
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
chmod +x src/factory_bent_project/*.sh`}</Code>
          <h3>2 · Set up each terminal</h3>
          <Code>{`export GZ_VERSION=harmonic
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash`}</Code>
          <h3>3 · Run</h3>
          <Code>{`cd ~/ros2_ws/src/factory_bent_project
./run_factory.sh`}</Code>
        </div>
        <div>
          <h3>What happens</h3>
          <ol className="steps">
            <li>The <strong>Product Order</strong> window opens. Pick products, set “Goes to” and the spacing, then press <em>Start simulation with this order</em>.</li>
            <li>Gazebo starts with both bents, the arm, the cameras, the attacher and the controller.</li>
            <li>Products appear on bent 1 and the arm starts picking. After about 8 s the order window opens again for live orders.</li>
            <li>Press <kbd>Ctrl</kbd>+<kbd>C</kbd> in the terminal to stop everything.</li>
          </ol>
          <h3>Variants</h3>
          <div className="table-wrap">
            <table>
              <tbody>
                <tr><td>Reuse the last order</td><td><code>./run_factory.sh --no-select</code></td></tr>
                <tr><td>Another world</td><td><code>./run_factory.sh --world straight-circular</code></td></tr>
                <tr><td>Different movement style</td><td><code>./run_factory.sh motion_type:=cycle</code></td></tr>
                <tr><td>No OpenCV windows</td><td><code>./run_factory.sh show_window:=false</code></td></tr>
                <tr><td>Build first</td><td><code>./run_factory.sh --build</code></td></tr>
              </tbody>
            </table>
          </div>
        </div>
      </div>

      <h3 className="sub-head reveal">Project reports</h3>
      <div className="grid three reveal">
        {reports.map(([f, t, d]) => (
          <a key={f} className="card report" href={report(f)} target="_blank" rel="noreferrer">
            <span className="pdf">PDF</span>
            <strong>{t}</strong>
            <span className="muted small">{d}</span>
          </a>
        ))}
      </div>

      <h3 className="sub-head reveal">Roadmap</h3>
      <ul className="roadmap reveal">
        {roadmap.map(([done, t]) => (
          <li key={t} className={done ? 'done' : ''}><span>{done ? '✓' : '○'}</span>{t}</li>
        ))}
      </ul>
    </Section>
  )
}

export default function App() {
  useReveal()
  return (
    <LightboxProvider>
      <Header />
      <main>
        <Hero />
        <Overview />
        <Architecture />
        <Worlds />
        <Vision />
        <Arm />
        <Products />
        <Pipeline />
        <Lab />
        <Timeline />
        <Run />
      </main>
      <footer className="footer">
        <div className="container footer-inner">
          <p>
            Developed by <strong>Hüseyin Tozluyurt</strong> · ROS 2 Jazzy · Gazebo Harmonic · OpenCV
          </p>
          <p>
            <a href={REPO_URL} target="_blank" rel="noreferrer">Source on GitHub</a> · <a href="#top">Back to top ↑</a>
          </p>
        </div>
      </footer>
    </LightboxProvider>
  )
}
