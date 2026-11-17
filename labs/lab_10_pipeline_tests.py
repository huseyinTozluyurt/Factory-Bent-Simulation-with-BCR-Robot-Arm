"""
LAB 10_pipeline_tests: Pipeline test cases, products through chains of arms to bins, bents and docks
Created 2026-11-01.

Each test case puts known products on S1 and lets chosen arm stations work with a routing table
(by the detector's label). Some products go into the stations' trash / recycling bins, the others
are placed on the next bent, where the next arm picks them and places them on the bent after that,
until they reach a dock box (or a conveyor no working arm takes them from).

For every product the test works out, from the same routing applied to its TRUE type, which arms
must move it and where it must end; it PASSES when the product really went that way.

    S1 -A1-> C1 -A2-> S2 -A4-> C2 -A6-> S4 (dock 1 cardboard)
                                 -A7-> S5 (dock 2 containers)
               -A3-> S3 -A5-> C3 -A8-> S6 -A10-> C4 -A11-> S8 (dock 4 express)
                                                    -A12-> S9 (dock 5 standard)
                                 -A9-> S7 (dock 3 large)

Run:  ./run_vision_lab.sh sim --world pipeline                    (terminal 1)
      ./run_vision_lab.sh run 10_pipeline_tests                   (terminal 2: all cases)
Without Gazebo:  ./run_vision_lab.sh run 10_pipeline_tests --fake --belt-speed 0.8
Some cases only:  PIPELINE_TESTS=bins_at_A1,chain_to_large_dock ./run_vision_lab.sh run 10_pipeline_tests --fake
Arms and belts only (route on the true type, not the detector):  PIPELINE_DECIDE_BY=truth ...
The summary is also written to labs/results/10_pipeline_tests/<time>/test_report.txt.

Add your own case: copy one PipelineCase below and change items / routing.
routing per station: "bent2", "trash", "recycling", "skip", a table {label: destination, "*": rest},
"rules" (the station's rule from pipeline_layout.py) or a function f(ctx) -> destination.
"""
import os

from factory_bringup.vision_lab.pipeline_testing import PipelineCase, run_cases, summary

LAB_TITLE = "Pipeline test cases: chains of arms to bins, bents and docks"
WORLD = "pipeline"
DETECTOR = "opencv_contours"
DETECTOR_OPTIONS = {"classify": True}
ARM_PARAMS = {}

CASES = [
    PipelineCase(
        name="one_hop_A1",
        description="A1 moves every product from S1 to ring C1 (no arm takes them from C1)",
        items=["cereal_box", "coffee_can", "shoe_box"],
        routing={"A1": "bent2"},
        timeout=180,
    ),
    PipelineCase(
        name="bins_at_A1",
        description="A1: cereal box -> trash bin, glass jar -> recycling bin, the rest -> C1",
        items=["cereal_box", "glass_jar", "coffee_can"],
        routing={"A1": {"cereal_box": "trash", "glass_jar": "recycling", "*": "bent2"}},
        timeout=180,
    ),
    PipelineCase(
        name="two_hops_to_S3",
        description="A1 S1->C1, A3 C1->S3; products wait at the end of S3 (A5 is off)",
        items=["shoe_box", "basketball"],
        routing={"A1": "bent2", "A3": "bent2"},
        timeout=240,
    ),
    PipelineCase(
        name="chain_to_large_dock",
        description="4 hops: A1 S1->C1, A3 C1->S3, A5 S3->C3, A9 C3->S7, then into dock 3",
        items=["toy_box", "coffee_can"],
        routing={"A1": "bent2", "A3": "bent2", "A5": "bent2", "A9": "bent2"},
        timeout=360,
    ),
    PipelineCase(
        name="branch_on_C1",
        description="A2 takes glass jars and coffee cans to S2 -> A4 -> C2 -> A7 -> dock 2; "
                    "A3 takes the rest to S3 -> A5 -> C3 -> A9 -> dock 3",
        items=["glass_jar", "shoe_box", "coffee_can", "basketball"],
        routing={"A1": "bent2", "A2": {"glass_jar": "bent2", "coffee_can": "bent2", "*": "skip"},
                 "A3": "bent2", "A4": "bent2", "A5": "bent2", "A7": "bent2", "A9": "bent2"},
        timeout=420,
    ),
    PipelineCase(
        name="bins_along_the_chain",
        description="A3 recycles pizza boxes, A5 throws coffee cans away, the rest reaches dock 3",
        items=["pizza_box", "coffee_can", "toy_box", "shoe_box"],
        routing={"A1": "bent2", "A3": {"pizza_box": "recycling", "*": "bent2"},
                 "A5": {"coffee_can": "trash", "*": "bent2"}, "A9": "bent2"},
        timeout=420,
    ),
    PipelineCase(
        name="longest_route_to_S9",
        description="6 hops: S1 -A1-> C1 -A3-> S3 -A5-> C3 -A8-> S6 -A10-> C4 -A12-> S9, then dock 5",
        items=["cereal_box", "tool_case"],
        routing={"A1": "bent2", "A3": "bent2", "A5": "bent2", "A8": "bent2", "A10": "bent2", "A12": "bent2"},
        timeout=600,
    ),
    PipelineCase(
        name="whole_factory_rules",
        description="stress test: all 12 stations follow their rules from pipeline_layout.py (recyclable, "
                    "cardboard, not_large, small ...); every product must reach the dock its type and size lead "
                    "to. A selective arm that misses a product hands it to the next arm, so this case needs "
                    "every camera to keep up (with --fake on a small PC it can fail 1-2 products)",
        items=["glass_jar", "basketball", "pizza_box", "teddy_bear", "phone_box", "tool_case"],
        routing={a: "rules" for a in ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", "A9", "A10", "A11", "A12"]},
        belt_speed=0.5,                  # 12 cameras at once: a selective arm must see every product pass
        spacing=2.0,
        timeout=900,
    ),
]


def main(lab):
    if not hasattr(lab, "all_stations"):
        raise SystemExit("These tests are for the pipeline world: add --world pipeline (or --fake)")
    only = [s.strip() for s in os.environ.get("PIPELINE_TESTS", "").split(",") if s.strip()]
    decide_by = os.environ.get("PIPELINE_DECIDE_BY", "label")
    print(f"Pipeline tests: {len(only) or len(CASES)} case(s), decisions by {decide_by}")
    results = run_cases(lab, CASES, only=only, decide_by=decide_by)
    text = summary(results)
    print("\n" + "=" * 78 + "\n" + text)
    os.makedirs(lab.output_dir, exist_ok=True)
    from factory_bringup.vision_lab.pipeline_testing import format_result
    with open(os.path.join(lab.output_dir, "test_report.txt"), "w") as f:
        f.write(text + "\n\n" + "\n\n".join(format_result(r) for r in results) + "\n")
    print(f"report: {os.path.join(lab.output_dir, 'test_report.txt')}")
