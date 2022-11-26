from .EcobotClientAPI import EcobotAPI

def main():
    ecobot_s40_hall10 = EcobotAPI(prefix='http://10.8.0.44:8080', debug=True)

    print("Testing EcobotAPI:")
    print(f"position(): {ecobot_s40_hall10.position()}")

if __name__ == '__main__':
    main()
