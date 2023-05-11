from fastapi import FastAPI 
from fastapi.responses import StreamingResponse 

app = FastAPI()

@app.get('/') 
async def main():
    def iter_file():
        with open("livestream/output.mp4", mode="rb") as stream_file:
            yield from stream_file
    # return StreamingResponse(iter_file(), media_type='text/event-stream')
    return StreamingResponse(iter_file(), media_type='video/mp4')