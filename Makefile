NAME := yuiskw/path-planning

build-docker:
	docker build --rm -t $(NAME) .

run-docker:
	docker run --rm -p 4567:4567 -t $(NAME)

kill-docker:
	docker rm -f $(shell docker ps -aq) >/dev/null 2>&1 || true

docker-machine-ssh-tunnel:
	docker-machine ssh default -L 4567:localhost:4567

