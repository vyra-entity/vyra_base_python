import redis

r = redis.Redis(decode_responses=True)
pubsub = r.pubsub()
pubsub.subscribe("my-channel")

for message in pubsub.listen():
    print("Empfangen:", message)