# Your own products

Products added by users live here and join the 100 built-in ones of `product_catalog.py`:
they can be ordered onto the bents, sorted into the bins, and detected by the OpenCV
detectors, the colour classifier and the YOLO dataset / vision labs.

* `custom_products.json` - one entry per product (size, colour, shape, label, recyclable, mesh ...)
* `<type>/` - the product's copied 3D model (+ `.mtl` / textures) and photo

## Adding a product

**Order GUI** (`ros2 run factory_bringup order_gui`): under *Selected product* press
**Add your own product ...**, then choose a 3D model (`.stl`, `.obj`, `.dae`) or a simple
shape (box, cylinder, sphere) with a size, an optional photo and a colour. **Import .zip** /
**Export ...** share a product with someone else; **Remove** deletes it. The filter
*Own products* shows only yours.

**Command line**

```bash
python3 -m factory_bringup.custom_products add --name coffee_mug --label "Coffee mug" --mesh ~/models/mug.stl --size 0.12 0.09 0.10
python3 -m factory_bringup.custom_products add --name juice_box --shape box --size 0.10 0.06 0.20 --image ~/juice.jpg --recyclable
python3 -m factory_bringup.custom_products list
python3 -m factory_bringup.custom_products export coffee_mug ~/coffee_mug.zip
python3 -m factory_bringup.custom_products import ~/coffee_mug.zip
python3 -m factory_bringup.custom_products remove coffee_mug
```

(after `colcon build` also `ros2 run factory_bringup factory_products ...`)

## Rules

* At most 48 x 34 x 42 cm, at least 6 cm high, a top of at least 60 cm2 and long side / short
  side at most 4.5 (the gripper, the bins and the cameras are made for that).
* A 3D model is scaled to fit inside the size you give (proportions kept) and centred; tick
  "use the model's own size" only if it is modelled in metres. Its collision is the box (or
  cylinder) around it.
* The colour is what the overhead camera sees on top. It is adjusted automatically when the
  detectors could not see it: channels at most 0.6, not near black, not strongly green.
* The attacher and the pick and place pick up new products for the next order; restart the
  detector nodes / vision labs so their classes include them. Run "Take pictures in Gazebo"
  for a real picture of a 3D model in the GUI (a photo you add is used as the picture).
