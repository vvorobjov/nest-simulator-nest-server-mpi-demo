
## 1. Generate NESTML module

    docker compose -f docker-compose-nestml.yaml up

the output files should appear in `./nestml/target` directory, then change the ownership of the resulting files (or maybe you do not need or want to do that)

    sudo chown -R $USER:$USER nestml

## 2. Run the experiment with the NEST Server

    docker compose -f docker-compose-nest-pybullet.yaml up

the PyBullet simulation should appear with the skeleton arm moving

## 3. Run the experiment with the NEST Server MPI

    docker compose -f docker-compose-nest-mpi-pybullet.yaml up

the error should appear and the simulation crashes

```
nest-server-1       | Traceback (most recent call last):
nest-server-1       |   File "/usr/lib/python3/dist-packages/flask/app.py", line 2070, in wsgi_app
nest-server-1       |     response = self.full_dispatch_request()
nest-server-1       |   File "/usr/lib/python3/dist-packages/flask/app.py", line 1515, in full_dispatch_request
nest-server-1       |     rv = self.handle_user_exception(e)
nest-server-1       |   File "/usr/lib/python3/dist-packages/flask_cors/extension.py", line 165, in wrapped_function
nest-server-1       |     return cors_after_request(app.make_response(f(*args, **kwargs)))
nest-server-1       |   File "/usr/lib/python3/dist-packages/flask/app.py", line 1513, in full_dispatch_request
nest-server-1       |     rv = self.dispatch_request()
nest-server-1       |   File "/usr/lib/python3/dist-packages/flask/app.py", line 1499, in dispatch_request
nest-server-1       |     return self.ensure_sync(self.view_functions[rule.endpoint])(**req.view_args)
nest-server-1       |   File "/opt/nest/lib/python3.10/site-packages/nest/server/hl_api_server.py", line 310, in route_api_call
nest-server-1       |     return jsonify(response)
nest-server-1       |   File "/usr/lib/python3/dist-packages/flask/json/__init__.py", line 348, in jsonify
nest-server-1       |     f"{dumps(data, indent=indent, separators=separators)}\n",
nest-server-1       |   File "/usr/lib/python3/dist-packages/flask/json/__init__.py", line 129, in dumps
nest-server-1       |     rv = _json.dumps(obj, **kwargs)
nest-server-1       |   File "/usr/lib/python3.10/json/__init__.py", line 238, in dumps
nest-server-1       |     **kw).encode(obj)
nest-server-1       |   File "/usr/lib/python3.10/json/encoder.py", line 199, in encode
nest-server-1       |     chunks = self.iterencode(o, _one_shot=True)
nest-server-1       |   File "/usr/lib/python3.10/json/encoder.py", line 257, in iterencode
nest-server-1       |     return _iterencode(o, 0)
nest-server-1       |   File "/usr/lib/python3/dist-packages/flask/json/__init__.py", line 56, in default
nest-server-1       |     return super().default(o)
nest-server-1       |   File "/usr/lib/python3.10/json/encoder.py", line 179, in default
nest-server-1       |     raise TypeError(f'Object of type {o.__class__.__name__} '
nest-server-1       | TypeError: Object of type int64 is not JSON serializable
nest-server-1       | [2025-04-25 12:16:28,508] INFO in _internal: 172.24.0.3 - - [25/Apr/2025 12:16:28] "POST /api/GetStatus HTTP/1.1" 500 -
```
