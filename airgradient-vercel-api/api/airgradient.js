import { MongoClient } from 'mongodb'

const uri = "mongodb+srv://doadmin:u2317Rk0T46xFv9a@db-mongodb-MUI-station-497493fa.mongo.ondigitalocean.com/admin?tls=true&authSource=admin&replicaSet=db-mongodb-MUI-station"

export default async function handler(req, res) {
  // CORS headers
  res.setHeader('Access-Control-Allow-Origin', '*')
  res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS')
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type')

  if (req.method === 'OPTIONS') {
    return res.status(200).end()
  }

  if (req.method === 'POST') {
    try {
      const client = new MongoClient(uri)
      await client.connect()
      
      const db = client.db('airgradient')
      const collection = db.collection('readings')
      
      const result = await collection.insertOne({
        ...req.body,
        timestamp: new Date(),
        receivedAt: new Date().toISOString()
      })
      
      await client.close()
      
      return res.status(200).json({ 
        success: true, 
        id: result.insertedId 
      })
    } catch (error) {
      console.error('Database error:', error)
      return res.status(500).json({ 
        success: false, 
        error: error.message 
      })
    }
  }
  
  return res.status(405).json({ error: 'Method not allowed' })
}