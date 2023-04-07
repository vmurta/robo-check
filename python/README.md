# Python Model of Triangle Intersection

This folder describes the math behind triangle intersection (used for the narrow phase).

To install, within the `python/` director, run
```
python3 -m venv env
source env/bin/activate
pip install -r requirements.txt
```

Run the tests with
```
python3 -m pytest .
```

The algorithm described is adapted from [here](https://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf).
