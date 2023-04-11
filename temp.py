import asyncio

async def coroutine1():
    async for i in range(5):
        print(f'Coroutine 1: {i}')
        await asyncio.sleep(1)

async def coroutine2():
    async for i in range(5):
        print(f'Coroutine 2: {i}')
        await asyncio.sleep(2)

loop = asyncio.get_event_loop()

# coroutine1과 coroutine2를 이벤트 루프에 등록하고 실행
tasks = [loop.create_task(coroutine1()), loop.create_task(coroutine2())]
loop.run_until_complete(asyncio.gather(*tasks))

# 이벤트 루프 종료
loop.close()
